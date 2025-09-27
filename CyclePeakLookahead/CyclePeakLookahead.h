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
    // --- Pins ---
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;               // CV output (compressor gain)
    AudioOutPin pinMaxInputCycle_;    // Max of current input cycle
    AudioOutPin pinMaxDelayedCycle_;  // Max of *delayed* output cycle
    FloatInPin  pinThreshold_;
    FloatInPin  pinRatio_;
    FloatInPin  pinAttack_;
    FloatInPin  pinRelease_;

    // --- Buffers ---
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;

    // --- Input cycle tracking ---
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;
    int   lastPositiveWidth_;
    int   minCycleGuard_;

    // --- Delayed cycle tracking ---
    float lastDelayedSample_;     // last delayed sample
    float delayedCyclePeak_;      // peak in current delayed cycle
    float delayedCyclePeakHold_;  // value held until next delayed cycle
    int   delayedSamplesSinceCycleStart_;

    // --- Quantisation ---
    bool  useCeil_ = true;
    bool  useFloor_ = true;
    float quantStep_ = 0.1f;

    // --- Misc ---
    double sampleRate_;
};
