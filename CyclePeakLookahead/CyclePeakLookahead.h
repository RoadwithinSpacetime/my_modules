#pragma once

#include "../mp_sdk_audio.h"
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);
    void open() override;
    void subProcess(int bufferOffset, int sampleFrames) override;
    void onSetPins() override;
private:
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;
    FloatInPin pinLookaheadMs_;
    FloatInPin pinHysteresis_;
    IntInPin   pinAbsMode_;

    double sampleRate_ = 0.0; // always set from DAW in open()
    int64_t inputSamplePos_ = 0;
    int64_t outputSamplePos_ = 0;

    int maxLookaheadSamples_ = 0;
    int lookaheadSamples_ = 0;

    std::vector<float> delay_;
    size_t delayWrite_ = 0;
    size_t delayRead_ = 0;

    float lastSample_ = 0.0f;
    bool haveLast_ = false;
    float currentCycleMax_ = 0.0f;
    int64_t currentCycleStartPos_ = 0;

    struct CycleInfo { int64_t startPos; float peak; };
    std::deque<CycleInfo> cycles_;

    float hysteresis_ = 0.001f;
    bool absMode_ = true;

    void updateLookahead();
    bool zeroCrossingNegToPos(float prev, float now, float hysteresis) const;
};
