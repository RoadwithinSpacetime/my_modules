#include "RwSSaturation.h"
#include <algorithm>
#include <cstring>

#undef min
#undef max

RwSSaturation::RwSSaturation()
    : harmFirPos_(0), hystFirPos_(0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
    initializePin(pinAlpha_);
    initializePin(pinBeta_);
    initializePin(pinHyst_);
    initializePin(pinHystFreq_);
}

int32_t RwSSaturation::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.0) sampleRate_ = 44100.0;

    harmFirTaps_ = 31;
    hystFirTaps_ = 31;

    // harmonic path filter before x²/x³
    makeFIR(harmFirTaps_, sampleRate_, 15000.0, harmFirCoeffs_);

    // hysteresis FIR default 15 kHz
    makeFIR(hystFirTaps_, sampleRate_, 15000.0, hystFirCoeffs_);

    harmFirBuf_.assign(harmFirTaps_, 0.0f);
    hystFirBuf_.assign(hystFirTaps_, 0.0f);

    harmFirPos_ = 0;
    hystFirPos_ = 0;

    x2_dc_ = 0.0f;
    const double dcTau = 0.05;
    x2_dc_alpha_ = static_cast<float>(1.0 - std::exp(-1.0 / (sampleRate_ * dcTau)));

    hystState_ = 0.0f;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);

    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    // update hysteresis filter if cutoff pin changes
    double hystCutoff = std::clamp(pinHystFreq_.getValue(), 1000.0f, 20000.0f);
    makeFIR(hystFirTaps_, sampleRate_, hystCutoff, hystFirCoeffs_);

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
}

void RwSSaturation::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    if (out)
        std::memset(out, 0, sampleFrames * sizeof(float));

    float* in = getBuffer(pinIn_);
    if (in && pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
        subProcess(sampleFrames);
    }
}

// FIR design
void RwSSaturation::makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs)
{
    coeffs.resize(taps);
    const int M = taps - 1;
    const double fc = cutoffHz / sampleRate;

    for (int n = 0; n < taps; ++n)
    {
        int k = n - M / 2;
        double h = (k == 0) ? 2.0 * fc : std::sin(2.0 * M_PI * fc * k) / (M_PI * k);
        double w = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / M);
        coeffs[n] = static_cast<float>(h * w);
    }

    double sum = 0.0;
    for (auto v : coeffs) sum += v;
    if (sum != 0.0)
        for (auto& v : coeffs) v = static_cast<float>(v / sum);
}

// FIR process (circular buffer)
float RwSSaturation::firProcess(std::vector<float>& buf, int& pos, const std::vector<float>& coeffs, float input)
{
    buf[pos] = input;
    double acc = 0.0;
    int idx = pos;
    const int N = static_cast<int>(coeffs.size());
    for (int i = 0; i < N; ++i)
    {
        acc += coeffs[i] * buf[idx];
        --idx;
        if (idx < 0) idx += N;
    }
    pos = (pos + 1) % N;
    return static_cast<float>(acc);
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out) return;

    const float drive = std::max(0.0f, pinDrive_.getValue());
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alpha = pinAlpha_.getValue();
    const float beta = pinBeta_.getValue();
    const float hyst = std::clamp(pinHyst_.getValue(), 0.0f, 1.0f);

    const float effAlpha = alpha * alphaScale_;
    const float effBeta = beta * betaScale_;
    const float trim = 1.0f / (1.0f + (drive - 1.0f)); // soft compensation

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x_in = in[s] * drive;

        // shared pre-filter for harmonic path
        float x_pre = this->firProcess(harmFirBuf_, harmFirPos_, harmFirCoeffs_, x_in);

        // generate harmonics from pre-filtered signal
        float x2 = x_pre * x_pre;
        float x3 = x_pre * x_pre * x_pre;

        // DC removal for x²
        x2_dc_ += x2_dc_alpha_ * (x2 - x2_dc_);
        float x2_hp = x2 - x2_dc_;

        // combine harmonics
        float harm2 = effAlpha * x2_hp;
        float harm3 = effBeta * x3;

        float shaped = x_in - harm2 - harm3;

        // hysteresis smoothing
        hystState_ += hyst * (shaped - hystState_);
        float wet = this->firProcess(hystFirBuf_, hystFirPos_, hystFirCoeffs_, hystState_);

        // mix and trim
        float outSample = ((1.0f - mix) * in[s]) + (mix * wet);
        outSample *= trim;

        if (!std::isfinite(outSample)) outSample = 0.0f;
        outSample = std::clamp(outSample, -20.0f, 20.0f);
        out[s] = outSample;
    }
}

// Register
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
