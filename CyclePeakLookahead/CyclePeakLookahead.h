#pragma once

#include "mp_sdk_audio.h"
#include <vector>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    void open() override;                 // SDK3 uses void, not int32_t
    void subProcess(int bufferOffset, int sampleFrames); // no override
    void onSetPins() override;

private:
    void updateLookahead();

    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;
    FloatInPin  pinLookaheadMs_;
    FloatInPin  pinHysteresis_;
    IntInPin    pinAbsMode_;

    float sampleRate_ = 44100.0f;
    int lookaheadSamples_ = 0;
    int maxLookaheadSamples_ = 0;
    float hysteresis_ = 0.0f;
    bool absMode_ = false;

    std::vector<float> delay_;
    size_t delayWrite_ = 0;
    size_t delayRead_ = 0;
    float currentPeak_ = 0.0f;
};
