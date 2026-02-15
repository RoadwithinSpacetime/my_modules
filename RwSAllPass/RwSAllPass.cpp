#include "RwSAllPass.h"
#include "mp_sdk_factory.h"

#include <cstring>

// Phase anchor frequencies (Fairchild-style spacing)
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

    env_ = 0.0f;
    updateBaseCoefficients();

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

    if (!pinInL_.isStreaming() || !pinInR_.isStreaming())
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
// Base coefficient setup
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

        // Conservative analog-like scaling
        g *= (0.12f + 0.28f * depthCurve);

        float a = (1.0f - g) / (1.0f + g);

        // Absolute stability guarantee
        if (a > 0.999f) a = 0.999f;
        if (a < -0.999f) a = -0.999f;

        aBase_[i] = a;

        apL_[i].reset();
        apR_[i].reset();
    }

    env_ = 0.0f;
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

    if (pinInL_.isStreaming() && pinInR_.isStreaming())
    {
        env_ = 0.0f;
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
    // CRITICAL: never touch buffers unless streaming
    if (!pinInL_.isStreaming() || !pinInR_.isStreaming())
        return;

    const float* inL = getBuffer(pinInL_);
    const float* inR = getBuffer(pinInR_);
    float* outL = getBuffer(pinOutL_);
    float* outR = getBuffer(pinOutR_);

    if (!inL || !inR || !outL || !outR)
        return;

    const float attack = 1.0f / (sampleRate_ * 0.08f);
    const float release = 1.0f / (sampleRate_ * 0.7f);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float l = inL[s];
        float r = inR[s];

        // Envelope follower (energy, not RMS)
        float level = 0.5f * (fabsf(l) + fabsf(r));
        if (level > env_)
            env_ += attack * (level - env_);
        else
            env_ += release * (level - env_);

        float motion = tanhf(env_ * 2.2f);

        for (int i = 0; i < STAGES; ++i)
        {
            float freqWeight = 1.0f / (1.0f + phaseFreqs[i] / 900.0f);

            float a = aBase_[i] * (1.0f + 0.04f * motion * freqWeight);

            // Hard safety clamp (must stay)
            if (a > 0.999f) a = 0.999f;
            if (a < -0.999f) a = -0.999f;

            apL_[i].a = a;
            apR_[i].a = a;

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
