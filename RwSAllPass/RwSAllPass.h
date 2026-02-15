#pragma once
#include "mp_sdk_audio.h"
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace gmpi;

static const int STAGES = 4;

// =======================
// First-order all-pass mono
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
        x1 = 0.0f;
        y1 = 0.0f;
    }
};

// =======================
// RwSAllPass (mono DSP)
// =======================
class RwSAllPass : public MpBase2
{
public:
    RwSAllPass();

    int32_t open() override;
    void onSetPins() override;

private:
    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);
    void updateBaseCoefficients();

    // Pins (IDENTICAL pattern to RwSSaturation)
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatInPin  pinDepth_;

    // DSP
    std::array<AllPassStage, STAGES> ap_;
    std::array<float, STAGES> aBase_;
    std::array<float, STAGES> aDyn_;

    double sampleRate_ = 44100.0;

    // Internal envelope
    float env_ = 0.0f;

    // Startup stability
    float startupGain_ = 0.0f;
};
