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

    // Add this line if you want CPU-saving silent mode:
    void subProcessSilent(int sampleFrames);

private:
    //=== Pins ===
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;
    FloatInPin  pinThreshold_;
    FloatInPin  pinRatio_;

    //=== Lookahead audio delay ===
    std::vector<float> lookaheadBuffer_;
    int bufferWritePos_ = 0;
    int lookaheadSamples_ = 0;

    //=== Per-cycle peak detection ===
    float lastSample_ = 0.0f;
    float cyclePeak_ = 0.0f;
    float previousCyclePeak_ = 0.0f;
    int samplesSinceCycleStart_ = 0;

    //=== Adaptive zero-crossing guard ===
    int lastPositiveWidth_ = 0;
    int minCycleGuard_ = 0;

    //=== Misc ===
    double sampleRate_ = 0.0;

    //=== CV smoothing ===
    float cvStart_ = 1.0f;
    float cvTarget_ = 1.0f;
    int cycleLength_ = 1;
    int cyclePos_ = 0;
};
