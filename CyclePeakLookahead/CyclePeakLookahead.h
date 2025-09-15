#pragma once
#include "mp_sdk_audio.h"
#include <cmath>
#include <vector>

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
public:
    CyclePeakLookahead();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;       // delayed audio out
    AudioOutPin pinCV_;        // CV out (normalized 0..1)
    FloatInPin  pinThreshold_; // threshold (0..1 control -> mapped in code)
    FloatInPin  pinRatio_;     // ratio (1..20)

    // Lookahead buffers (audio + CV)
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;      // parallel CV delay line
    int bufferWritePos_;
    int lookaheadSamples_; // 30 ms default

    // Cycle tracking (input side)
    float lastSample_;
    float cyclePeak_;          // running peak for current input cycle
    float previousCyclePeak_;  // peak of the cycle that just finished
    int samplesSinceCycleStart_;

    // Adaptive zero-crossing guard
    int lastPositiveWidth_;
    int minCycleGuard_;

    // Misc
    double sampleRate_;
};
