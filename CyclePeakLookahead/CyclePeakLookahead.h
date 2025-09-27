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
    AudioOutPin pinCV_;             // Compressor-style CV
    AudioOutPin pinMaxInputCycle_;  // Smoothed max peak of input cycle
    AudioOutPin pinMaxDelayedCycle_;// Smoothed max peak of delayed cycle
    FloatInPin  pinThreshold_;      // Threshold (0.0–1.0 mapped to 0–10 V)
    FloatInPin  pinRatio_;          // Ratio (1:1 .. 20:1)

    // --- Buffers ---
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_; // ~30 ms in samples

    // --- Cycle tracking ---
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;
    int   lastPositiveWidth_;
    int   minCycleGuard_;

    // --- Peak smoothing (input cycle) ---
    float prevPeakSmoothIn_;
    float nextPeakSmoothIn_;
    int   rampSamplesTotalIn_;
    int   rampSamplesRemainingIn_;

    // --- Peak smoothing (delayed cycle) ---
    float prevPeakSmoothDelay_;
    float nextPeakSmoothDelay_;
    int   rampSamplesTotalDelay_;
    int   rampSamplesRemainingDelay_;

    // --- Misc ---
    double sampleRate_;
};
