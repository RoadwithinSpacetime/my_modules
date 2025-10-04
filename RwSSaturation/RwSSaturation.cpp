#include "RwSSaturation.h"
#include <cstring>
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

    // --- Design a simple 9-tap low-pass sinc FIR (~4 kHz cutoff)
    int taps = 9;
    firCoeffs_.resize(taps);
    firBuffer_.assign(taps, 0.0f);
    firPos_ = 0;

    double fc = 4000.0 / (sampleRate_ / 2.0); // normalized cutoff
    int M = taps - 1;
    for (int n = 0; n < taps; ++n)
    {
        int k = n - M / 2;
        if (k == 0)
            firCoeffs_[n] = (float)fc;
        else
            firCoeffs_[n] = (float)(sin(M_PI * fc * k) / (M_PI * k));
        // Hamming window
        firCoeffs_[n] *= (0.54f - 0.46f * cosf(2.0f * (float)M_PI * n / M));
    }
    // Normalize gain
    float sum = 0.0f;
    for (auto c : firCoeffs_) sum += c;
    for (auto& c : firCoeffs_) c /= sum;

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
        memset(out, 0, sampleFrames * sizeof(float));
}

float RwSSaturation::processFIR(float input)
{
    int taps = (int)firCoeffs_.size();
    firBuffer_[firPos_] = input;

    float y = 0.0f;
    int idx = firPos_;
    for (int i = 0; i < taps; ++i)
    {
        y += firCoeffs_[i] * firBuffer_[idx];
        idx = (idx - 1 + taps) % taps;
    }

    firPos_ = (firPos_ + 1) % taps;
    return y;
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out) return;

    float drive = (std::max)(0.0f, pinDrive_.getValue());
    float mix = (std::clamp)(pinMix_.getValue(), 0.0f, 1.0f);
    float alpha = pinAlpha_.getValue(); // cubic
    float beta = pinBeta_.getValue();  // quadratic
    float hyst = pinHyst_.getValue();

    for (int s = 0; s < sampleFrames; ++s)
    {
        // Apply drive
        float x = in[s] * drive;

        // Filtered harmonics
        float x2f = processFIR(x * x);
        float x3f = processFIR(x * x * x);

        // Normalize harmonic contributions (prevent overpowering linear path)
        // Adjust constants as needed for musical scaling
        float harmonic2 = alpha * 0.10f * x2f;  // 2nd harmonic, normalized
        float harmonic3 = beta * 0.25f * x3f;  // 3rd harmonic, normalized

        // Combine with linear
        float shaped = x - harmonic2 - harmonic3;

        // Hysteresis smoothing
        hystState_ += hyst * (shaped - hystState_);
        float saturated = hystState_;

        // Dry/Wet mix
        out[s] = (1.0f - mix) * in[s] + mix * saturated;
    }
}

// Register
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
