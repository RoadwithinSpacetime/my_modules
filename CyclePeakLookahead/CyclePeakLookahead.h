#pragma once
#include "mp_sdk_audio.h"
#include <cmath>
#include <vector>
#include <algorithm>

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
public:
    CyclePeakLookahead();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;
    FloatInPin  pinThreshold_;   // Threshold (0.0–1.0 mapped to 0–10 V)
    FloatInPin  pinRatio_;       // Ratio (1:1 .. 20:1)
    IntInPin    pinRampLength_;  // Ramp length in samples

    // Buffers
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_ = 0;
    int lookaheadSamples_ = 0;

    // Cycle tracking
    float lastSample_ = 0.0f;
    float cyclePeak_ = 0.0f;
    float previousCyclePeak_ = 0.0f;
    int   samplesSinceCycleStart_ = 0;
    int   lastPositiveWidth_ = 0;
    int   minCycleGuard_ = 0;

    // Ramp control
    int   rampLength_ = 10;
    bool  rampActive_ = false;
    bool  firstCycle_ = true;
    float prevCvValue_ = 1.0f;
    float nextCvValue_ = 1.0f;
    int   rampSamplesRemaining_ = 0;

    // Ceil quantisation
    bool  useCeil_ = true;   // enable upward quantisation
    float ceilStep_ = 0.1f;  // step size (0.1 = quantise to 0.1 steps)

    double sampleRate_ = 0.0;
};
