#pragma once
#include "mp_sdk_audio.h"
#include <cmath>

using namespace gmpi;

class RwSSaturation : public MpBase2
{
public:
    RwSSaturation();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames);

private:
    // --- Pins ---
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;

    FloatInPin pinDrive_;     // Input gain
    FloatInPin pinMix_;       // Dry/Wet

    // --- Saturation state ---
    float Bsat_ = 1.0f;    // saturation limit
    float alpha_ = 2.0f;   // curve steepness
    float hyst_ = 0.02f;   // hysteresis blend
    float state_ = 0.0f;   // memory state

    inline float saturate(float x)
    {
        // Apply drive
        float H = x * alpha_;

        // Magnetization curve (tanh is smooth, keeps only low harmonics)
        float target = std::tanh(H) * Bsat_;

        // Apply hysteresis smoothing (like magnetic memory)
        state_ += hyst_ * (target - state_);

        return state_;
    }
};
