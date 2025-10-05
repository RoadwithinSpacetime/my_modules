#include "RwSSaturation.h"
#include <algorithm>
#include <cstring>

#undef max
#undef min

RwSSaturation::RwSSaturation()
    : firPos_(0), hystPos_(0)
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

    // --- Build harmonic FIR
    const double cutoffHz = 4000.0;
    makeFIR(firTaps_, sampleRate_, cutoffHz, firCoeffs_);
    x2Buf_.assign(firTaps_, 0.0f);
    x3Buf_.assign(firTaps_, 0.0f);
    firPos_ = 0;

    // --- Build hysteresis FIR
    hystCutoffHz_ = 4000.0;
    makeFIR(hystTaps_, sampleRate_, hystCutoffHz_, hystCoeffs_);
    hystBuf_.assign(hystTaps_, 0.0f);
    hystPos_ = 0;

    // --- DC filter for x²
    const double dcTauSec = 0.05;
    x2_dc_alpha_ = static_cast<float>(1.0 - std::exp(-1.0 / (sampleRate_ * dcTauSec)));
    x2_dc_ = 0.0f;

    hystState_ = 0.0f;
    isSilent_ = false;
    silenceCounter_ = 0;

    pinOut_.setStreaming(true);
    setSubProcess(&RwSSaturation::subProcess);
    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    if (pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
    }
    else
    {
        setSubProcess(&RwSSaturation::subProcessSilent);
        pinOut_.setStreaming(false);
    }
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
    for (auto c : coeffs) sum += c;
    for (auto& c : coeffs) c = static_cast<float>(c / sum);
}

float RwSSaturation::firProcess(std::vector<float>& buf, int pos, const std::vector<float>& coeffs, float input)
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
    return static_cast<float>(acc);
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out)
        return;

    const float drive = std::max(0.0f, pinDrive_.getValue());
    const float trim = (drive > 0.0001f) ? (1.0f / drive) : 1.0f;
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alpha = pinAlpha_.getValue();
    const float beta = pinBeta_.getValue();
    const float hyst = pinHyst_.getValue();
    const float hystCut = std::clamp(pinHystFreq_.getValue(), 200.0f, 16000.0f);

    const float effAlpha = alpha * alphaScale_;
    const float effBeta = beta * betaScale_;

    const int N = static_cast<int>(firCoeffs_.size());
    const int HN = static_cast<int>(hystCoeffs_.size());

    bool silentBlock = true;

    for (int s = 0; s < sampleFrames; ++s)
    {
        const float input = in[s];
        if (std::fabs(input) > 1e-6f)
            silentBlock = false;

        float x_lin = input * drive;

        float x2 = x_lin * x_lin;
        float x3 = x_lin * x_lin * x_lin;

        x2_dc_ += x2_dc_alpha_ * (x2 - x2_dc_);
        float x2_hp = x2 - x2_dc_;

        float x2f = firProcess(x2Buf_, firPos_, firCoeffs_, x2_hp);
        float x3f = firProcess(x3Buf_, firPos_, firCoeffs_, x3);
        firPos_ = (firPos_ + 1) % N;

        float harm2 = effAlpha * x2f;
        float harm3 = effBeta * x3f;
        float shaped = x_lin - harm2 - harm3;

        // Hysteresis FIR
        float hystOut = firProcess(hystBuf_, hystPos_, hystCoeffs_, shaped);
        hystPos_ = (hystPos_ + 1) % HN;

        hystState_ += hyst * (hystOut - hystState_);
        float wet = hystState_ * trim;

        float outSample = (1.0f - mix) * input + mix * wet;
        outSample = std::clamp(outSample, -20.0f, 20.0f);
        out[s] = outSample;
    }

    // --- Auto sleep handling
    if (silentBlock)
    {
        if (++silenceCounter_ > silenceThreshold_)
        {
            setSubProcess(&RwSSaturation::subProcessSilent);
            pinOut_.setStreaming(false);
            isSilent_ = true;
        }
    }
    else
    {
        silenceCounter_ = 0;
        isSilent_ = false;
    }
}

// Register
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
