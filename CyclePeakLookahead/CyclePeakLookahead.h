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

    // Misc
    double sampleRate_ = 0.0;
};
