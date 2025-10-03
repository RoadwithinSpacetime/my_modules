#pragma once
#include "mp_sdk_audio.h"
#include <cmath>
#include <algorithm>

using namespace gmpi;

class RwSSaturation : public MpBase2
{
public:
    RwSSaturation();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;

    FloatInPin  pinDrive_;   // Input gain / steepness
    FloatInPin  pinMix_;     // Dry/Wet (0..1)

    // --- Waveshaper ---
    float alphaBase_ = 0.5f;  // base steepness for cubic
    float hyst_ = 0.02f;      // hysteresis (0 = off)
    float state_ = 0.0f;      // memory state

    // --- Anti-alias lowpass (one-pole ~4 kHz) ---
    float lpState_ = 0.0f;
    float lpCoeff_ = 0.0f;

    inline float waveshape(float in, float alpha);
    inline float lowpass(float x);
};
