#include "RwSSaturation.h"
#include <algorithm>

#undef max
#undef min

RwSSaturation::RwSSaturation()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
    initializePin(pinAlpha_);
    initializePin(pinBeta_);
    initializePin(pinHyst_);
}

int32_t RwSSaturation::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.0) sampleRate_ = 44100.0;

    // Design shared FIR low-pass (~4 kHz)
    firTaps_ = 9;
    makeFIR(firTaps_, sampleRate_, 4000.0, firCoeffs_);
    x2Buf_.assign(firTaps_, 0.0f);
    x3Buf_.assign(firTaps_, 0.0f);
    firPos_ = 0;

    // DC removal (for x² path)
    const double dcTau = 0.05; // 50 ms
    x2_dc_alpha_ = static_cast<float>(1.0 - std::exp(-1.0 / (sampleRate_ * dcTau)));
    x2_dc_ = 0.0f;

    hystState_ = 0.0f;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
}

void RwSSaturation::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    if (out)
        std::memset(out, 0, sampleFrames * sizeof(float));
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

    // Normalize to unity gain at DC
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
        if (--idx < 0) idx = N - 1;
    }
    return static_cast<float>(acc);
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out) return;

    const float drive = std::max(0.0f, pinDrive_.getValue());
    const float trim = 1.0f / std::sqrt(std::max(0.0001f, drive)); // perceptual output gain
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alpha = pinAlpha_.getValue();
    const float beta = pinBeta_.getValue();
    const float hyst = pinHyst_.getValue();

    const float effBeta = beta * betaScale_;
    const float effAlpha = alpha * alphaScale_;
    const int N = static_cast<int>(firCoeffs_.size());

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s] * drive;

        // nonlinear terms
        float x2 = x * x;
        float x3 = x * x * x;

        // DC-remove x²
        x2_dc_ += x2_dc_alpha_ * (x2 - x2_dc_);
        float x2_hp = x2 - x2_dc_;

        // Filter harmonics
        int pos = firPos_;
        float x2f = firProcess(x2Buf_, pos, firCoeffs_, x2_hp);
        float x3f = firProcess(x3Buf_, pos, firCoeffs_, x3);
        firPos_ = (firPos_ + 1) % N;

        // Combine harmonic shaping
        float shaped = x - effBeta * x3f - effAlpha * x2f;

        // Hysteresis smoothing
        hystState_ += hyst * (shaped - hystState_);
        float wet = hystState_ * trim;

        // Mix dry/wet
        out[s] = (1.0f - mix) * in[s] + mix * wet;
    }
}

// Register module
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
