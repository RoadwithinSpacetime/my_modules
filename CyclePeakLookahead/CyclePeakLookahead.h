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
    AudioOutPin pinCV_;              // Compressor CV (0–1)
    AudioOutPin pinMaxInputCycle_;   // Peak of *input* cycle
    AudioOutPin pinMaxDelayedCycle_; // Same peak but delayed
    FloatInPin  pinThreshold_;
    FloatInPin  pinRatio_;

    // --- Buffers ---
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    std::vector<float> delayedPeakBuffer_; // delay line for maxDelayedCycle
    int bufferWritePos_;
    int lookaheadSamples_;           // ~30 ms delay

    // --- Cycle tracking ---
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;
    int   lastPositiveWidth_;
    int   minCycleGuard_;

    // --- Held values ---
    float maxInputCycle_;            // latched each input cycle end

    // --- Misc ---
    double sampleRate_;
};
