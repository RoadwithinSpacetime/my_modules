#pragma once

#include "SEModule.h"
#include <vector>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    int32_t open() override;
    void subProcess(int sampleFrames) override;
    void onSetPins() override;

private:
    void updateLookahead();

    MpPinAudio pinIn_;
    MpPinAudio pinOut_;
    MpPinFloat pinPeak_;
    MpPinFloat pinLookaheadMs_;
    MpPinFloat pinHysteresis_;
    MpPinInt pinAbsMode_;

    std::vector<float> delay_;
    int delayWrite_ = 0;
    int delayRead_ = 0;
    int lookaheadSamples_ = 0;
    int maxLookaheadSamples_ = 0;
    float currentPeak_ = 0.0f;
    float hysteresis_ = 0.0f;
    bool absMode_ = false;
    float sampleRate_ = 44100.0f;
};
