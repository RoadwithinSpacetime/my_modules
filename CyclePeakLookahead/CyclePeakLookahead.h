#pragma once
#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>

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
    AudioOutPin pinCV_;
    FloatInPin  pinThreshold_;
    FloatInPin  pinRatio_;
    FloatInPin  pinMaxFreq_;   // Hz – max update rate
    FloatInPin  pinAttack_;    // ms
    FloatInPin  pinRelease_;   // ms

    // Buffers
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;

    // Cycle detection
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;
    int   lastPositiveWidth_;
    int   minCycleGuard_;

    // Timing
    double sampleRate_;
    int    minSamplesBetweenUpdates_;
    int    samplesSinceLastUpdate_;

    // Smoothing
    float smoothedCV_;
};
