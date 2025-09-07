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

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinPeak_; // audio-rate, per-cycle held peak aligned to delayed audio

    // Lookahead audio delay
    std::vector<float> lookaheadBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_; // 30 ms in samples

    // Per-cycle peak hold buffer (aligned to delayed audio)
    std::vector<float> peakHoldBuffer_;

    // Cycle tracking
    float lastSample_;
    float cyclePeak_;
    int samplesSinceCycleStart_;

    // Adaptive zero-crossing guard
    int lastPositiveWidth_; // length of previous positive half-cycle
    int minCycleGuard_;     // quarter of that length

    // Misc
    double sampleRate_;
};
