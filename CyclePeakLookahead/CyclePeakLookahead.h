#pragma once

#include "mp_sdk_audio.h"
#include "mp_sdk_common.h"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
    // Pins
    AudioInPin pinIn;
    AudioOutPin pinOut;

    FloatOutPin pinPeak;
    FloatInPin pinLookaheadMs;
    FloatInPin pinHysteresis;
    IntInPin   pinAbsMode;

    // Internal state
    double sampleRate_;
    size_t maxLookaheadSamples_;
    size_t lookaheadSamples_;

    size_t delayWrite_;
    size_t delayRead_;
    std::vector<float> delay_;

    float cyclePeak_;
    float lastSample_;
    float hysteresis_;
    bool  absMode_;

public:
    CyclePeakLookahead();

    void onSetPins() override;
    void subProcess(int sampleFrames);
    int32_t open() override;

private:
    void updateLookahead();
};
