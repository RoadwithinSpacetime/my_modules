#pragma once
#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>

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
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatInPin  pinDrive_;   // gain / drive
    FloatInPin  pinMix_;     // dry/wet mix (0..1)
    FloatInPin  pinAlpha_;   // cubic strength
    FloatInPin  pinHyst_;    // hysteresis amount

    // Filter state (one-pole lowpass ~4 kHz)
    float lp_a0_ = 0.0f;
    float lp_b1_ = 0.0f;
    float lp_z1_ = 0.0f;

    // Hysteresis state
    float hystState_ = 0.0f;

    // Internal
    double sampleRate_ = 44100.0;
};
