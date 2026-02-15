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
        x1 = 0.0f;
        y1 = 0.0f;
    }
};

// =======================
// RwSAllPass (stereo, streaming-safe)
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

    // Audio pins
    AudioInPin  pinInL_;
    AudioInPin  pinInR_;
    AudioOutPin pinOutL_;
    AudioOutPin pinOutR_;

    // Control
    FloatInPin  pinDepth_;

    // DSP
    std::array<AllPassStage, STAGES> apL_;
    std::array<AllPassStage, STAGES> apR_;
    std::array<float, STAGES> aBase_;
    std::array<float, STAGES> aDyn_;

    double sampleRate_ = 44100.0;

    // Envelope follower
    float env_ = 0.0f;

    // Startup stabilization
    float startupGain_ = 0.0f;
};
