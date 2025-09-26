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
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;        // Control Voltage out (0–10 V)
    FloatInPin  pinThreshold_; // Threshold (0.0–1.0 mapped to 0–10 V)
    FloatInPin  pinRatio_;     // Ratio (1:1 .. 20:1)

    // Lookahead audio delay
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;          // decimated CV buffer (audio rate)
    int bufferWritePos_;
    int lookaheadSamples_;                  // 30 ms in samples

    // Cycle tracking
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int   samplesSinceCycleStart_;
    int   lastPositiveWidth_;
    int   minCycleGuard_;

    // --- 4× CV oversampling ---
    static const int oversampleFactor_ = 4;
    std::vector<float> cvOversampleBuffer_; // CV values at 4× audio rate
    int cvOversamplePos_;
    float cvFilterState_;                   // one-pole lowpass state
    float cvFilterCoeff_;                   // coefficient for decimation filter

    // Misc
    double sampleRate_;
};
