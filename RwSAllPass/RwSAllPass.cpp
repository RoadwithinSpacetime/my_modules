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

    env_ = 0.0f;
    startupGain_ = 0.0f;

    updateBaseCoefficients();

    setSubProcess(&RwSAllPass::subProcess);
    pinOutL_.setStreaming(true);
    pinOutR_.setStreaming(true);

    return MpBase2::open();
}

// =======================
// Pin changes
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
// Coefficient setup + state reset
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

    if (outL)
        std::memset(outL, 0, sampleFrames * sizeof(float));
    if (outR)
        std::memset(outR, 0, sampleFrames * sizeof(float));

    if (pinInL_.isStreaming() && pinInR_.isStreaming())
    {
        startupGain_ = 0.0f;
        env_ = 0.0f;
        setSubProcess(&RwSAllPass::subProcess);
        pinOutL_.setStreaming(true);
        pinOutR_.setStreaming(true);
    }
}

// =======================
// Audio processing (CRITICAL GUARD INCLUDED)
// =======================
void RwSAllPass::subProcess(int sampleFrames)
{
    // ABSOLUTELY REQUIRED
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
    const float fadeIn = 1.0f / (sampleRate_ * 0.02f);

    for (int s = 0; s < sampleFrames; ++s)
    {
        // Startup fade
        if (startupGain_ < 1.0f)
        {
            startupGain_ += fadeIn;
            if (startupGain_ > 1.0f)
                startupGain_ = 1.0f;
        }

        float l = inL[s];
        float r = inR[s];

        // Envelope (mono energy)
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

        // Implicit dry anchor (DC safety)
        l = inL[s];
        r = inR[s];

        outL[s] = l * startupGain_;
        outR[s] = r * startupGain_;
    }
}

// =======================
// Registration
// =======================
namespace
{
    auto r = Register<RwSAllPass>::withId(L"RwSAllPass");
}
