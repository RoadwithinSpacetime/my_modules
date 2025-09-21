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
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;        // CV out (0–10 V)
    FloatInPin  pinThreshold_; // Threshold (0–1 mapped to 0–10 V)
    FloatInPin  pinRatio_;     // Ratio (1:1 .. 20:1)

    // Buffers
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_{};
    int lookaheadSamples_{};

    // Cycle tracking
    float lastSample_{};
    float cyclePeak_{};
    float previousCyclePeak_{};
    int   samplesSinceCycleStart_{};

    int lastPositiveWidth_{};
    int minCycleGuard_{};

    // Ramp
    int   rampLength_{ 10 };      // user ramp length in samples
    int   rampSamplesRemaining_{};
    float prevCvValue_{ 1.0f };
    float nextCvValue_{ 1.0f };

    double sampleRate_{};
};
