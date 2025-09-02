#pragma once

#include "mp_sdk_audio.h"
#include "mp_sdk_common.h"
#include <vector>
#include <cmath>
#include <algorithm>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    int32_t open() override;
    void subProcess(int bufferOffset, int sampleFrames);
    void onSetPins() override;

private:
    void updateLookahead();

    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;
    FloatInPin  pinLookaheadMs_;
    FloatInPin  pinHysteresis_;
    IntInPin    pinAbsMode_;

    // Delay buffer
    std::vector<float> delay_;
    size_t delayWrite_ = 0;
    size_t delayRead_ = 0;

    // Peak tracking
    float currentPeak_ = 0.0f;
    int lookaheadSamples_ = 0;
    int maxLookaheadSamples_ = 0;

    // Settings
    float sampleRate_ = 44100.0f;
    float hysteresis_ = 0.0f;
    bool absMode_ = false;
};
