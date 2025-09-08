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
    AudioOutPin pinCV_;        // control voltage output (0..10 V)
    FloatInPin  pinThreshold_; // threshold in volts
    FloatInPin  pinRatio_;     // compression ratio (1..20)

    // Lookahead audio delay
    std::vector<float> lookaheadBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_; // 30 ms in samples

    // Peak tracking aligned with delayed audio
    std::vector<float> peakHoldBuffer_;

    // Cycle tracking
    float lastSample_;
    float cyclePeak_;
    int samplesSinceCycleStart_;

    // Adaptive zero-crossing guard
    int lastPositiveWidth_;
    int minCycleGuard_;

    // Misc
    double sampleRate_;
};
