#pragma once

#include "mp_sdk_audio.h"   // SDK3 base
#include <vector>
#include <algorithm>
#include <cmath>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    int32_t open() override;
    void onSetPins() override;

private:
    void updateLookahead();
    void subProcessAudio(int sampleFrames);  // SDK3 subProcess with single argument

    // Pins
    MpAudioInPin pinIn_;
    MpAudioOutPin pinOut_;
    MpFloatOutPin pinPeak_;
    MpFloatInPin pinLookaheadMs_;
    MpFloatInPin pinHysteresis_;
    MpBoolInPin pinAbsMode_;

    // Internal
    std::vector<float> delay_;
    int delayWrite_ = 0;
    int delayRead_ = 0;
    int lookaheadSamples_ = 0;
    int maxLookaheadSamples_ = 0;
    float sampleRate_ = 44100.0f;
    float currentPeak_ = 0.0f;
    float hysteresis_ = 0.0f;
    bool absMode_ = false;
};
