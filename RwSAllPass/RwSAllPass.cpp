#include "RwSAllPass.h"
#include "mp_sdk_factory.h"

#include <cstring>

// Phase anchor frequencies mono
static const float phaseFreqs[STAGES] =
{
    90.0f, 300.0f, 1000.0f, 3200.0f
};

// =======================
// Constructor
// =======================
RwSAllPass::RwSAllPass()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
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

    startupGain_ = 0.0f;
    env_ = 0.0f;

    updateBaseCoefficients();

    for (int i = 0; i < STAGES; ++i)
        ap_[i].reset();

    setSubProcess(&RwSAllPass::subProcess);
    pinOut_.setStreaming(true);

    return MpBase2::open();
}

// =======================
// Pin changes
// =======================
void RwSAllPass::onSetPins()
{
    updateBaseCoefficients();

    if (!pinIn_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOut_.setStreaming(false);
        return;
    }

    setSubProcess(&RwSAllPass::subProcess);
    pinOut_.setStreaming(true);
}

// =======================
// Coefficient setup + RESET
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

        // Reset filter state (critical)
        ap_[i].reset();
    }

    // Reset envelope (critical)
    env_ = 0.0f;
}

// =======================
// Silent processing
// =======================
void RwSAllPass::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);

    if (out)
        std::memset(out, 0, sampleFrames * sizeof(float));

    if (pinIn_.isStreaming())
    {
        startupGain_ = 0.0f;
        env_ = 0.0f;
        setSubProcess(&RwSAllPass::subProcess);
        pinOut_.setStreaming(true);
    }
}

// =======================
// Audio processing
// =======================
void RwSAllPass::subProcess(int sampleFrames)
{
    if (!pinIn_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOut_.setStreaming(false);
        return;
    }

    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);

    if (!in || !out)
        return;

    const float attack = 1.0f / (sampleRate_ * 0.08f);
    const float release = 1.0f / (sampleRate_ * 0.7f);
    const float fadeIn = 1.0f / (sampleRate_ * 0.02f);

    for (int s = 0; s < sampleFrames; ++s)
    {
        // Startup fade-in
        if (startupGain_ < 1.0f)
        {
            startupGain_ += fadeIn;
            if (startupGain_ > 1.0f)
                startupGain_ = 1.0f;
        }

        float x = in[s];

        // Envelope follower
        float level = fabsf(x);
        if (level > env_)
            env_ += attack * (level - env_);
        else
            env_ += release * (level - env_);

        float motion = tanhf(env_ * 3.0f);

        for (int i = 0; i < STAGES; ++i)
        {
            float freqWeight = 1.0f / (1.0f + phaseFreqs[i] / 700.0f);
            aDyn_[i] = aBase_[i] * (1.0f + 0.06f * motion * freqWeight);

            ap_[i].a = aDyn_[i];
            x = ap_[i].process(x);
        }

        // Implicit dry anchor (DC safety)
        x = 0.995f * x + 0.005f * in[s];

        out[s] = x * startupGain_;
    }
}

// =======================
// Registration
// =======================
namespace
{
    auto r = Register<RwSAllPass>::withId(L"RwSAllPass");
}
