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
    int   bufferWritePos_;
    int   lookaheadSamples_; // 30 ms in samples

    // Scheduled per-cycle peak (aligned with delayed audio)
    std::vector<float> peakHoldBuffer_;

    // Cycle tracking
    float  lastSample_;
    float  cyclePeak_;           // max of current cycle
    float  previousCyclePeak_;   // peak of previous cycle
    int    samplesSinceCycleStart_; // samples since last zero-crossing

    // Misc
    double sampleRate_;
};
