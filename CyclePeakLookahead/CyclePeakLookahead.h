#pragma once

#include "mp_sdk_audio.h"
#include "mp_sdk_common.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    int32_t open() override;
    void onSetPins() override;

private:
    void subProcess(int bufferOffset, int sampleFrames);
    void updateLookahead();

    // --- Pins ---
    AudioInPin   pinIn_;
    AudioOutPin  pinOut_;
    FloatOutPin  pinPeak_;
    FloatInPin   pinLookaheadMs_;
    FloatInPin   pinHysteresis_;
    IntInPin     pinAbsMode_;

    // --- State ---
    double sampleRate_ = 44100.0;
    size_t maxLookaheadSamples_ = 0;
    size_t lookaheadSamples_ = 0;

    std::vector<float> delay_;
    size_t delayWrite_ = 0;
    size_t delayRead_ = 0;

    // cycle peak detection
    float cyclePeak_ = 0.0f;
    float lastSample_ = 0.0f;
    float hysteresis_ = 0.001f;
    bool absMode_ = false;
};
