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
    AudioOutPin pinCV_;             // Control Voltage out (0–10 V)
    AudioOutPin pinMaxInputCycle_;  // NEW: max peak of current input cycle
    AudioOutPin pinMaxDelayedCycle_; // NEW: max peak of delayed/buffered output
    FloatInPin  pinThreshold_;      // Threshold (0.0–1.0 mapped to 0–10 V)
    FloatInPin  pinRatio_;          // Ratio (1:1 .. 20:1)
    FloatInPin  pinAttack_;         // Attack time in ms
    FloatInPin  pinRelease_;        // Release time in ms

    // --- Buffers ---
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;

    // --- Cycle tracking ---
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int samplesSinceCycleStart_;
    int lastPositiveWidth_;
    int minCycleGuard_;

    // --- Quantisation ---
    bool  useCeil_ = true;
    bool  useFloor_ = true;
    float quantStep_ = 0.1f;

    // --- Misc ---
    double sampleRate_;
};
