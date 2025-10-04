#pragma once
#include "mp_sdk_audio.h"
#include <vector>
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
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatInPin  pinDrive_;     // pre-gain (>= 0)
    FloatInPin  pinMix_;       // dry/wet 0..1
    FloatInPin  pinAlpha_;     // cubic strength base
    FloatInPin  pinHyst_;      // hysteresis smoothing (0..1)
    FloatInPin  pinThreshold_; // threshold for saturation (linear scale, 0..1). default ~0.9

    // GSinc FIR (9 taps) for LP on cubic term
    std::vector<float> gsincCoeffs_;
    std::vector<float> cubicBuf_;
    int cubicBufPos_;
    const int gsincTaps_ = 9;

    // Hysteresis state
    float hystState_;

    // Internal
    double sampleRate_;
};
