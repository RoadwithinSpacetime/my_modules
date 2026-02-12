#pragma once

#include "mp_sdk_audio.h"
#include "mp_sdk_controller.h"

#define STAGES 16

// =======================
// One-pole all-pass stage
// =======================
struct AllPassStage
{
    float a;
    float x1;
    float y1;

    AllPassStage() : a(0.f), x1(0.f), y1(0.f) {}

    inline float process(float x)
    {
        float y = -a * x + x1 + a * y1;
        x1 = x;
        y1 = y;
        return y;
    }
};

// =======================
// Module class
// =======================
class RwSAllPass : public MpBase
{
public:
    RwSAllPass(IMpUnknown* host);

    void process(int sampleFrames);
    void onSetPins();

private:
    // ---- Audio pins ----
    AudioInPin  pinInL;
    AudioInPin  pinInR;
    AudioOutPin pinOutL;
    AudioOutPin pinOutR;

    // ---- Control pins ----
    FloatInPin pinDepth;
    FloatInPin pinDrift;
    BoolInPin  pinBypass;

    // ---- DSP ----
    AllPassStage apL[STAGES];
    AllPassStage apR[STAGES];

    void updateCoefficients();
};
