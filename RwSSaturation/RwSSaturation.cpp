#include "RwSSaturation.h"

REGISTER_PLUGIN(RwSSaturation, "RwSSaturation");

RwSSaturation::RwSSaturation()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
}

int32_t RwSSaturation::open()
{
    MpBase2::open();

    // 4 kHz lowpass for cubic term
    double sampleRate = getSampleRate();
    double cutoff = 4000.0;
    lpCoeff_ = 1.0 - std::exp(-2.0 * M_PI * cutoff / sampleRate);

    return gmpi::MP_OK;
}

void RwSSaturation::onSetPins()
{
    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
}

float RwSSaturation::lowpass(float x)
{
    lpState_ += lpCoeff_ * (x - lpState_);
    if (!std::isfinite(lpState_)) lpState_ = 0.0f;
    return lpState_;
}

float RwSSaturation::waveshape(float in, float alpha)
{
    // Linear path
    float lin = in;

    // Cubic path with lowpass smoothing
    float cubic = in * in * in;
    cubic = lowpass(cubic);

    // Combine
    return lin + alpha * cubic;
}

void RwSSaturation::subProcess(int sampleFrames)
{
    float* inBuf = getBuffer(pinIn_);
    float* outBuf = getBuffer(pinOut_);
    if (!inBuf || !outBuf)
        return;

    float drive = std::clamp(pinDrive_.getValue(), 0.01f, 20.0f);
    float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    float alpha = alphaBase_ * drive;

    for (int i = 0; i < sampleFrames; ++i)
    {
        float dry = inBuf[i];

        // Apply drive
        float driven = dry * drive;
        driven = std::clamp(driven, -5.0f, 5.0f);

        // Waveshaper with LP on cubic term
        float shaped = waveshape(driven, alpha);

        // Hysteresis smoothing
        state_ += hyst_ * (shaped - state_);
        if (!std::isfinite(state_)) state_ = 0.0f;

        float wet = state_;

        // Dry/wet mix
        outBuf[i] = dry * (1.0f - mix) + wet * mix;
    }
}
