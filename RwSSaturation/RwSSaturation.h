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
    FloatInPin  pinAlpha_;   // cubic harmonic (3rd)
    FloatInPin  pinBeta_;    // quadratic harmonic (2nd)
    FloatInPin  pinHyst_;    // hysteresis smoothing

    // FIR filter (9-tap sinc low-pass ~4 kHz)
    std::vector<float> firCoeffs_;
    std::vector<float> firBuffer_;
    int firPos_ = 0;

    float processFIR(float input);

    // Hysteresis state
    float hystState_ = 0.0f;

    // Internal
    double sampleRate_ = 44100.0;
};
