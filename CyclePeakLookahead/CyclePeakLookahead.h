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
    AudioOutPin pinOut_;       // delayed audio out
    AudioOutPin pinCV_;        // CV output (0..1 -> 0..10 V in SE)
    FloatInPin  pinThreshold_; // threshold 0..1 (mapped to 0..10 V)
    FloatInPin  pinRatio_;     // compression ratio (1:1..20:1)

    // Look-ahead buffer
    std::vector<float> lookaheadBuffer_;
    int bufferWritePos_{};
    int lookaheadSamples_{};

    // Cycle tracking
    float lastSample_{};
    float cyclePeak_{};
    float previousCyclePeak_{};

    double sampleRate_{};
};
