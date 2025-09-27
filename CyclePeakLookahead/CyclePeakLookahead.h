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
    // --- Pins ---
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;             // Compressor CV (0–1)
    AudioOutPin pinMaxInputCycle_;  // NEW : peak of current input cycle
    AudioOutPin pinMaxDelayedCycle_;// NEW : delayed (lookahead) peak
    FloatInPin  pinThreshold_;      // Threshold (0–1 mapped to 0–10 V)
    FloatInPin  pinRatio_;          // Ratio (1:1 .. 20:1)

    // --- Buffers ---
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    std::vector<float> peakHoldBuffer_; // buffer to delay the held peak
    int bufferWritePos_;
    int lookaheadSamples_;          // ~30 ms

    // --- Cycle tracking ---
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;
    int   lastPositiveWidth_;
    int   minCycleGuard_;

    // --- Held peaks ---
    float inputCyclePeakHold_;      // latched at each input cycle end

    // --- Misc ---
    double sampleRate_;
};
