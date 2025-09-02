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
    double sampleRate_;
    size_t maxLookaheadSamples_;
    size_t lookaheadSamples_;

    std::vector<float> delay_;
    size_t delayWrite_;
    size_t delayRead_;

    float cyclePeak_;
    float lastSample_;
    float hysteresis_;
    bool absMode_;
};
