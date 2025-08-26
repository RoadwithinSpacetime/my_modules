#pragma once

#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>
#include <algorithm>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    int32_t open() override;
    void subProcess(int bufferOffset) override;
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

    // State
    std::vector<float> delay_;
    size_t delayWrite_ = 0;
    size_t delayRead_ = 0;
    int lookaheadSamples_ = 0;
    int maxLookaheadSamples_ = 0;
    float hysteresis_ = 0.0f;
    bool absMode_ = false;
    float currentPeak_ = 0.0f;
    float sampleRate_ = 44100.0f;
};
