#pragma once
#include "mp_sdk_audio.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    FloatInPin  pinDrive_; // Drive amount
    FloatInPin  pinMix_;   // Dry/Wet mix

    // --- State ---
    float state_ = 0.0f;     // hysteresis state
    float hyst_ = 0.02f;     // hysteresis smoothing
    float lpState_ = 0.0f;   // lowpass state (for cubic term)
    float lpCoeff_ = 0.0f;   // lowpass coefficient
    float alphaBase_ = 0.5f; // cubic weighting

    // --- Helpers ---
    float lowpass(float x);
    float waveshape(float in, float alpha);
};
