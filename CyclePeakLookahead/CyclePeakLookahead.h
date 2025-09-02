#pragma once

#include "mp_sdk_audio.h"
#include "mp_sdk_common.h"
#include <vector>
#include <cmath>
#include <algorithm>

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    int32_t open() override;
    void onSetPins() override;

private:
    // --- Processing ---
    void subProcess(int bufferOffset, int sampleFrames);
    void updateLookahead();

    // --- Pins ---
    AudioInPin   pinIn_;
    AudioOutPin  pinOut_;
    FloatOutPin  pinPeak_;
    FloatInPin   pinLookaheadMs_;
    FloatInPin   pinHysteresis_;
    IntInPin     pinAbsMode_;

    // --- State ---
    double sampleRate_{ 44100.0 };
    int maxLookaheadSamples_{ 0 };
    int lookaheadSamples_{ 0 };

    std::vector<float> delay_;
    int delayWrite_{ 0 };
    int delayRead_{ 0 };

    // --- Cycle Peak Detection ---
    float lastSample_{ 0.0f };
    float cyclePeak_{ 0.0f };

    // --- Options ---
    float hysteresis_{ 0.0f }; // (still included if you want it later)
    bool absMode_{ false };
};

