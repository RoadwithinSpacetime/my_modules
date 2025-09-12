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
    AudioOutPin pinCV_;
    FloatInPin  pinThreshold_;
    FloatInPin  pinRatio_;

    // Lookahead delay
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_; // NEW: CV delay line
    int bufferWritePos_;
    int lookaheadSamples_;

    // Cycle tracking
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int samplesSinceCycleStart_;

    // Adaptive zero-crossing guard
    int lastPositiveWidth_;
    int minCycleGuard_;

    // Misc
    double sampleRate_;

    // CV per cycle
    float cvTarget_; // updated at zero-crossing, delayed to match audio
};
