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
    AudioOutPin pinCV_;           // Control Voltage out (0–10 V)
    FloatInPin  pinThreshold_;    // Threshold (0.0–1.0 mapped to 0–10 V)
    FloatInPin  pinRatio_;        // Ratio (1:1 .. 20:1)
    FloatInPin  pinAttack_;       // Attack time in ms
    FloatInPin  pinRelease_;      // Release time in ms
    FloatInPin  pinMaxFundamental_; // NEW: maximum fundamental frequency to react to (Hz)

    // Lookahead audio delay
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_; // 30 ms in samples

    // Cycle tracking
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;

    // Adaptive zero-crossing guard
    int lastPositiveWidth_; // length of previous positive half-cycle
    int minCycleGuard_;     // quarter of that length

    // NEW: minimum cycle length allowed (derived from max fundamental)
    float maxFundamentalHz_;
    int   minCycleSamples_;

    // Misc
    double sampleRate_;
};
