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
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;        // Control Voltage out (0–10 V)
    FloatInPin  pinThreshold_; // Threshold (0.0–1.0 mapped to 0–10 V)
    FloatInPin  pinRatio_;     // Ratio (1:1 .. 20:1)

    // Lookahead buffer
    std::vector<float> lookaheadBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;

    // Cycle tracking (input side)
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int samplesSinceCycleStart_;
    int lastPositiveWidth_;
    int minCycleGuard_;

    // CV control
    float currentCv_;       // CV applied to output (delayed promotion)
    float nextCvTarget_;    // CV calculated from input, waiting to be promoted
    int   promoteIndex_;    // Buffer index at which to promote nextCvTarget

    double sampleRate_;
};
