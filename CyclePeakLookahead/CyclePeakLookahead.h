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
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;         // CV output based on threshold and ratio
    FloatInPin pinThreshold_; // Threshold in volts
    FloatInPin pinRatio_;     // Ratio (1:1..20:1)

    // Lookahead audio delay
    std::vector<float> lookaheadBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_; // 30 ms in samples

    // Per-cycle peak tracking
    std::vector<float> peakHoldBuffer_;
    float lastSample_;
    float cyclePeak_;
    int samplesSinceCycleStart_;

    // Adaptive zero-crossing guard
    int lastPositiveWidth_;
    int minCycleGuard_;

    // Misc
    double sampleRate_;
};
