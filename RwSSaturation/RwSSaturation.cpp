#include "RwSSaturation.h"
#include <algorithm>
#include <cstring>

#undef min
#undef max

RwSSaturation::RwSSaturation()
    : harmFirPos1_(0), harmFirPos2_(0), hystFirPos_(0)
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
    if (sampleRate_ <= 0.0)
        sampleRate_ = 44100.0;

    harmFirTaps_ = 9;
    hystFirTaps_ = 31;

    // harmonic filter (used before x³ generation)
    makeFIR(harmFirTaps_, sampleRate_, 4000.0f, harmFirCoeffs_);

    // hysteresis FIR default 20000 Hz
    makeFIR(hystFirTaps_, sampleRate_, 20000.0f, hystFirCoeffs_);

    harmFirBuf1_.assign(harmFirTaps_, 0.0f);
    harmFirBuf2_.assign(harmFirTaps_, 0.0f);
    hystFirBuf_.assign(hystFirTaps_, 0.0f);

    harmFirPos1_ = 0;
    harmFirPos2_ = 0;
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
    // update hysteresis filter cutoff
    float hystCutoff = std::clamp(
        static_cast<float>(pinHystFreq_.getValue()),
        1000.0f,
        20000.0f
    );
    makeFIR(hystFirTaps_, sampleRate_, hystCutoff, hystFirCoeffs_);

    // CPU save mode
    if (!pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcessSilent);
        pinOut_.setStreaming(false);
        return;
    }

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
}

void RwSSaturation::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    if (out)
        std::memset(out, 0, sampleFrames * sizeof(float));

    if (pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
    }
}

void RwSSaturation::makeFIR(int taps, double sampleRate, float cutoffHz, std::vector<float>& coeffs)
{
    coeffs.resize(taps);
    const int M = taps - 1;
    const double fc = static_cast<double>(cutoffHz) / sampleRate;

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
        if (idx < 0)
            idx += N;
    }
    pos = (pos + 1) % N;
    return static_cast<float>(acc);
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out)
        return;

    const float drive = std::max(0.0f, pinDrive_.getValue());
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alpha = pinAlpha_.getValue();
    const float beta = pinBeta_.getValue();
    const float hyst = std::clamp(pinHyst_.getValue(), 0.0f, 1.0f);

    const float effAlpha = alpha * alphaScale_;
    const float effBeta = beta * betaScale_;
    const float trim = 1.0f / (1.0f + (drive - 1.0f));

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x_in = in[s] * drive;

        // Apply hysteresis smoothing first
        hystState_ += hyst * (x_in - hystState_);
        float hystOut = this->firProcess(hystFirBuf_, hystFirPos_, hystFirCoeffs_, hystState_);

        // 3rd harmonic prefilter (shared for both harmonic stages)
        float x_pre = this->firProcess(harmFirBuf1_, harmFirPos1_, harmFirCoeffs_, hystOut);
        x_pre = this->firProcess(harmFirBuf2_, harmFirPos2_, harmFirCoeffs_, x_pre);

        // harmonic generation
        float x2 = x_pre * x_pre;
        float x3 = x_pre * x_pre * x_pre;

        // remove DC from x²
        x2_dc_ += x2_dc_alpha_ * (x2 - x2_dc_);
        float x2_hp = x2 - x2_dc_;

        float harm2 = effAlpha * x2_hp;
        float harm3 = effBeta * x3;

        float shaped = x_in - harm2 - harm3;

        // mix dry/wet
        float outSample = ((1.0f - mix) * in[s]) + (mix * shaped);
        outSample *= trim;

        if (!std::isfinite(outSample))
            outSample = 0.0f;
        outSample = std::clamp(outSample, -20.0f, 20.0f);

        out[s] = outSample;
    }

    // enter silent mode if input stops
    if (!pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcessSilent);
        pinOut_.setStreaming(false);
    }
}

namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
