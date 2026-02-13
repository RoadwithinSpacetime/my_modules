#include "RwSAllPass.h"
#include <algorithm>
#include <cstring>
#include <cmath>

#undef min
#undef max

// =======================
// Frequency layout (log-spaced, gentle)
// =======================
static const float baseFreqs[STAGES] =
{
     60.f,   95.f,  150.f,  230.f,
    350.f,  520.f,  750.f, 1100.f,
   1600.f, 2300.f, 3300.f, 4700.f,
   6600.f, 9200.f, 12500.f, 16000.f
};

// =======================
// Constructor
// =======================
RwSAllPass::RwSAllPass()
{
    initializePin(pinInL_);
    initializePin(pinInR_);
    initializePin(pinOutL_);
    initializePin(pinOutR_);

    initializePin(pinDepth_);
    initializePin(pinBypass_);
}

// =======================
// open()
// =======================
int32_t RwSAllPass::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.0f)
        sampleRate_ = 44100.0f;

    updateCoefficients();

    setSubProcess(&RwSAllPass::subProcess);
    pinOutL_.setStreaming(true);
    pinOutR_.setStreaming(true);

    return MpBase2::open();
}

// =======================
// onSetPins()
// IMPORTANT: NO streaming logic here
// =======================
void RwSAllPass::onSetPins()
{
    updateCoefficients();
}

// =======================
// Coefficient update (STABLE all-pass)
// =======================
void RwSAllPass::updateCoefficients()
{
    float depth = std::clamp(pinDepth_.getValue(), 0.0f, 1.0f);

    for (int i = 0; i < STAGES; ++i)
    {
        float freq = baseFreqs[i];

        float omega = 2.0f * float(M_PI) * freq / sampleRate_;
        float g = tanf(omega * 0.5f);

        // Gentle analog-style scaling
        g *= (0.25f + 0.75f * depth);

        float a = (1.0f - g) / (1.0f + g);

        apL_[i].a = a;
        apR_[i].a = a;

        apL_[i].reset();
        apR_[i].reset();
    }
}

// =======================
// Silent processing
// =======================
void RwSAllPass::subProcessSilent(int sampleFrames)
{
    float* outL = getBuffer(pinOutL_);
    float* outR = getBuffer(pinOutR_);

    if (outL) std::memset(outL, 0, sampleFrames * sizeof(float));
    if (outR) std::memset(outR, 0, sampleFrames * sizeof(float));

    // Wake up when audio appears
    if (pinInL_.isStreaming() || pinInR_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcess);
        pinOutL_.setStreaming(true);
        pinOutR_.setStreaming(true);
    }
}

// =======================
// Audio processing
// =======================
void RwSAllPass::subProcess(int sampleFrames)
{
    const float* inL = getBuffer(pinInL_);
    const float* inR = getBuffer(pinInR_);
    float* outL = getBuffer(pinOutL_);
    float* outR = getBuffer(pinOutR_);

    if (!outL || !outR)
        return;

    // If no input, output silence
    if (!inL && !inR)
    {
        std::memset(outL, 0, sampleFrames * sizeof(float));
        std::memset(outR, 0, sampleFrames * sizeof(float));
        return;
    }

    bool bypass = pinBypass_.getValue();

    for (int s = 0; s < sampleFrames; ++s)
    {
        float l = inL ? inL[s] : 0.0f;
        float r = inR ? inR[s] : l;

        if (!bypass)
        {
            for (int i = 0; i < STAGES; ++i)
            {
                l = apL_[i].process(l);
                r = apR_[i].process(r);
            }
        }

        outL[s] = l;
        outR[s] = r;
    }

    // Enter silent mode ONLY after streaming stops
    if (!pinInL_.isStreaming() && !pinInR_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOutL_.setStreaming(false);
        pinOutR_.setStreaming(false);
    }
}

// =======================
// Registration (same as RwSSaturation)
// =======================
namespace
{
    auto r = Register<RwSAllPass>::withId(L"RwSAllPass");
}
