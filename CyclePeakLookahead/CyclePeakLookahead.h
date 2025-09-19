#pragma once
#include "mp_sdk_audio.h"
#include <cmath>
#include <vector>
#include <random>

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
    FloatInPin  pinRamp_;      // Ramp length in samples
    FloatInPin  pinRelease_;   // Minimum release in samples

    // Buffers
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;

    // Cycle tracking
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int samplesSinceCycleStart_;

    int lastPositiveWidth_;
    int minCycleGuard_;

    // Ramp control
    int rampLength_;
    float prevCvValue_;
    float nextCvValue_;
    int rampSamplesRemaining_;

    double sampleRate_;

    // Release / dither
    int releaseMin_;
    float ditherAmount_;
    std::mt19937 rng_;
    std::uniform_real_distribution<float> ditherDist_;
};
