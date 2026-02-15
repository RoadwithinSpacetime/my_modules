#include "RwSAllPass.h"
#include "mp_sdk_factory.h"

#include <cstring>

// Phase centers (very gentle, analog-like)
static const float phaseFreqs[STAGES] =
{
    80.0f, 250.0f, 800.0f, 2500.0f
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
    if (sampleRate_ <= 0.0)
        sampleRate_ = 44100.0;

    updateCoefficients();

    for (int i = 0; i < STAGES; ++i)
    {
        apL_[i].reset();
        apR_[i].reset();
    }

    setSubProcess(&RwSAllPass::subProcess);
    pinOutL_.setStreaming(true);
    pinOutR_.setStreaming(true);

    return MpBase2::open();
}

// =======================
// Pin updates
// =======================
void RwSAllPass::onSetPins()
{
    updateCoefficients();

    if (!pinInL_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOutL_.setStreaming(false);
        pinOutR_.setStreaming(false);
    }
}

// =======================
// Coefficient calculation
// =======================
void RwSAllPass::updateCoefficients()
{
    float depth = pinDepth_.getValue();
    if (depth < 0.0f) depth = 0.0f;
    if (depth > 1.0f) depth = 1.0f;

    float depthCurve = depth * depth;

    for (int i = 0; i < STAGES; ++i)
    {
        float omega = 2.0f * float(M_PI) * phaseFreqs[i] / float(sampleRate_);
        float g = tanf(omega * 0.5f);

        // very gentle rotation
        g *= (0.15f + 0.4f * depthCurve);

        aBase_[i] = (1.0f - g) / (1.0f + g);
    }
}

// =======================
// Silent processing
// =======================
void RwSAllPass::subProcessSilent(int sampleFrames)
{
    float* outL = getBuffer(pinOutL_);
    float* outR = getBuffer(pinOutR_);

    if (outL)
        std::memset(outL, 0, sampleFrames * sizeof(float));
    if (outR)
        std::memset(outR, 0, sampleFrames * sizeof(float));

    if (pinInL_.isStreaming())
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

    if (!inL || !outL || !outR)
        return;

    bool bypass = pinBypass_.getValue();

    for (int s = 0; s < sampleFrames; ++s)
    {
        float l = inL[s];
        float r = inR ? inR[s] : l;

        if (!bypass)
        {
            for (int i = 0; i < STAGES; ++i)
            {
                apL_[i].a = aBase_[i];
                apR_[i].a = aBase_[i];

                l = apL_[i].process(l);
                r = apR_[i].process(r);
            }
        }

        outL[s] = l;
        outR[s] = r;
    }
}

// =======================
// Registration
// =======================
namespace
{
    auto r = Register<RwSAllPass>::withId(L"RwSAllPass");
}
