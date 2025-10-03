#include "RwSSaturation.h"
#include <cstring>   // memset

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RwSSaturation::RwSSaturation()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
}

int32_t RwSSaturation::open()
{
    state_ = 0.0f;
    lpState_ = 0.0f;

    // Low-pass coeff for 4 kHz cutoff
    float sr = getSampleRate();
    float fc = 4000.0f;
    float x = expf(-2.0f * static_cast<float>(M_PI) * fc / sr);
    lpCoeff_ = 1.0f - x;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);

    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    bool streaming = pinIn_.isStreaming();
    pinOut_.setStreaming(streaming);

    if (streaming)
        setSubProcess(&RwSSaturation::subProcess);
    else
        setSubProcess(nullptr);
}

// cubic waveshaper: x + a * x3
inline float RwSSaturation::waveshape(float in, float alpha)
{
    return in + alpha * in * in * in;
}

// simple one-pole LPF
inline float RwSSaturation::lowpass(float x)
{
    lpState_ += lpCoeff_ * (x - lpState_);
    return lpState_;
}

void RwSSaturation::subProcess(int sampleFrames)
{
    float* inBuf = getBuffer(pinIn_);
    float* outBuf = getBuffer(pinOut_);
    if (!inBuf || !outBuf)
        return;

    float drive = (std::max)(0.01f, pinDrive_.getValue());
    float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);

    // Map drive to alpha steepness
    float alpha = alphaBase_ * drive;

    for (int i = 0; i < sampleFrames; ++i)
    {
        float dry = inBuf[i];

        // Apply drive pre-gain
        float driven = dry * drive;

        // Apply cubic waveshaper
        float shaped = waveshape(driven, alpha);

        // Apply hysteresis (magnetic smoothing)
        state_ += hyst_ * (shaped - state_);
        float wet = state_;

        // Apply low-pass filter
        wet = lowpass(wet);

        // Mix dry/wet
        outBuf[i] = dry * (1.0f - mix) + wet * mix;
    }
}

// Register plugin
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
