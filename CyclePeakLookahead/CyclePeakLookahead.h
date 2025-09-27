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
    void subProcessSilent(int sampleFrames);

private:
    // --- Pins ---
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;             // compressed CV
    AudioOutPin pinMaxInputCycle_;  // max peak of current input cycle
    AudioOutPin pinMaxDelayedCycle_;// max peak of delayed cycle
    FloatInPin  pinThreshold_;      // Threshold (0–1 mapped to 0–10 V)
    FloatInPin  pinRatio_;          // Ratio (1:1 .. 20:1)
    FloatInPin  pinAttack_;         // (unused but kept for compatibility)
    FloatInPin  pinRelease_;        // (unused but kept for compatibility)

    // --- Buffers ---
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    std::vector<float> maxBuffer_;  // stores cycle max for delayed output
    int bufferWritePos_ = 0;
    int lookaheadSamples_ = 0; // 30 ms in samples

    // --- Cycle tracking ---
    float lastSample_ = 0.0f;
    float cyclePeak_ = 0.0f;
    float previousCyclePeak_ = 0.0f;
    int   samplesSinceCycleStart_ = 0;
    int   lastPositiveWidth_ = 0;
    int   minCycleGuard_ = 0;

    // --- Delayed cycle max ---
    float inputCyclePeakHold_ = 0.0f; // live input-cycle max
    float delayedCyclePeakHold_ = 0.0f; // delayed-cycle max

    // --- Misc ---
    double sampleRate_ = 0.0;
};
