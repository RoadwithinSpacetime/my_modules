#include "RwSAllPass.h"
#include "mp_sdk_factory.h"

#include <cstring>

// Phase anchor frequencies
static const float phaseFreqs[STAGES] =
{
    90.0f, 300.0f, 1000.0f, 3200.0f
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
}

// =======================
// open()
// =======================
int32_t RwSAllPass::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.0)
        sampleRate_ = 44100.0;

    updateBaseCoefficients();

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
    updateBaseCoefficients();

    if (!pinInL_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOutL_.setStreaming(false);
        pinOutR_.setStreaming(false);
        return;
    }

    setSubProcess(&RwSAllPass::subProcess);
    pinOutL_.setStreaming(true);
    pinOutR_.setStreaming(true);
}

// =======================
// Coefficient setup
// =======================
void RwSAllPass::updateBaseCoefficients()
{
    float depth = pinDepth_.getValue();
    if (depth < 0.0f) depth = 0.0f;
    if (depth > 1.0f) depth = 1.0f;

    float depthCurve = depth * depth;

    for (int i = 0; i < STAGES; ++i)
    {
        float omega = 2.0f * float(M_PI) * phaseFreqs[i] / float(sampleRate_);
        float g = tanf(omega * 0.5f);

        g *= (0.12f + 0.35f * depthCurve);

        aBase_[i] = (1.0f - g) / (1.0f + g);
        aDyn_[i] = aBase_[i];
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
    if (!pinInL_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOutL_.setStreaming(false);
        pinOutR_.setStreaming(false);
        return;
    }

    const float* inL = getBuffer(pinInL_);
    const float* inR = getBuffer(pinInR_);
    float* outL = getBuffer(pinOutL_);
    float* outR = getBuffer(pinOutR_);

    if (!inL || !outL || !outR)
        return;

    // Envelope timing
    const float attack = 1.0f / (sampleRate_ * 0.08f);
    const float release = 1.0f / (sampleRate_ * 0.7f);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float l = inL[s];
        float r = inR ? inR[s] : l;

        // mono energy
        float level = 0.5f * (fabsf(l) + fabsf(r));

        if (level > env_)
            env_ += attack * (level - env_);
        else
            env_ += release * (level - env_);

        float motion = tanhf(env_ * 3.0f);

        for (int i = 0; i < STAGES; ++i)
        {
            float freqWeight = 1.0f / (1.0f + phaseFreqs[i] / 700.0f);
            aDyn_[i] = aBase_[i] * (1.0f + 0.06f * motion * freqWeight);

            apL_[i].a = aDyn_[i];
            apR_[i].a = aDyn_[i];

            l = apL_[i].process(l);
            r = apR_[i].process(r);
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
