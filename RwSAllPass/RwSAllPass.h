#pragma once

#include "mp_sdk_audio.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace gmpi;
#pragma once

#define STAGES 16

// =======================
// First-order all-pass
// =======================
struct AllPassStage
{
    float a = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;

    inline float process(float x)
    {
        float y = -a * x + x1 + a * y1;
        x1 = x;
        y1 = y;
        return y;
    }

    inline void reset()
    {
        x1 = y1 = 0.0f;
    }
};

// =======================
// Module
// =======================
class RwSAllPass : public MpBase2
{
public:
    RwSAllPass();

    int32_t open();
    void onSetPins();

    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinInL_;
    AudioInPin  pinInR_;
    AudioOutPin pinOutL_;
    AudioOutPin pinOutR_;

    FloatInPin pinDepth_;
    FloatInPin pinDrift_;
    BoolInPin  pinBypass_;

    // DSP
    AllPassStage apL_[STAGES];
    AllPassStage apR_[STAGES];

    float driftPhase_[STAGES] = {};

    float sampleRate_ = 44100.0f;

    void updateCoefficients();
};
