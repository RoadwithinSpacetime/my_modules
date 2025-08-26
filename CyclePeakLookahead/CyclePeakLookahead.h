#pragma once

#include "mp_sdk_audio.h"
#include <vector>

// A module that delays audio slightly and reports the cycle peak with lookahead
class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    int32_t open() override;
    void onSetPins() override;

private:
    // Main audio process callback
    void subProcess(int sampleFrames);

    void updateLookahead();

    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;
    FloatInPin  pinLookaheadMs_;
    FloatInPin  pinHysteresis_;
    IntInPin    pinAbsMode_;

    // Internal state
    float sampleRate_{ 44100.0f };
    int maxLookaheadSamples_{ 0 };
    int lookaheadSamples_{ 0 };
    int delayWrite_{ 0 };
    int delayRead_{ 0 };
    float hysteresis_{ 0.0f };
    bool absMode_{ false };
    float currentPeak_{ 0.0f };

    std::vector<float> delay_;
};
