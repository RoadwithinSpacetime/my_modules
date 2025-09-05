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
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinPeak_;       // audio-rate peak

    float lastSample_;
    float cyclePeak_;
    std::vector<float> lookaheadBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;      // 30 ms in samples
    double sampleRate_;
};
