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
    FloatInPin  pinAttack_;      // (unused, kept for compatibility)
    FloatInPin  pinRelease_;     // (unused, kept for compatibility)

    // Buffers
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;

    // Cycle tracking
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;
    int   lastPositiveWidth_;
    int   minCycleGuard_;

    double sampleRate_;

    // Quantisation
    float quantStep_ = 0.1f; // Ceil quantisation step (0.1 = 0.1 V steps)
    bool  useCeil_ = true; // enable/disable ceil quantisation
};
