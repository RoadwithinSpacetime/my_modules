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
    AudioOutPin pinCV_;        // Control Voltage out (0–10 V)
    FloatInPin  pinThreshold_; // Threshold (0.0–1.0 mapped to 0–10 V)
    FloatInPin  pinRatio_;     // Ratio (1:1 .. 20:1)
    FloatInPin  pinAttack_;    // Attack time in ms
    FloatInPin  pinRelease_;   // Release time in ms

    // --- Buffers ---
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_; // 30 ms in samples

    // --- Cycle tracking ---
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int samplesSinceCycleStart_;
    int lastPositiveWidth_;
    int minCycleGuard_;

    // --- Quantisation (ceil/floor) ---
    bool  useCeil_ = true;   // enable upward rounding
    bool  useFloor_ = true;  // enable downward rounding
    float quantStep_ = 0.01f; // step size (1 decimal, 0.1 V)

    // --- Misc ---
    double sampleRate_;
};