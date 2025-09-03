#pragma once
#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>

class CyclePeakLookahead : public MpBase2
{
public:
    CyclePeakLookahead();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int bufferOffset, int sampleFrames);

private:
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;
    FloatInPin pinLookaheadMs_;
    FloatInPin pinHysteresis_;
    IntInPin pinAbsMode_;

    double sampleRate_;
    float cyclePeak_;
    float lastSample_;
    float hysteresis_;
    bool absMode_;
};
