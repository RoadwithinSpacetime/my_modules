#include "RwSAllPass.h"
#include "mp_sdk_factory.h"

#include <algorithm>
#include <cstring>
#include <cmath>

#undef min
#undef max

// =======================
// Stage frequency layout
// =======================
static const float baseFreqs[STAGES] =
{
     60.f,   95.f,  150.f,  230.f,
    350.f,  520.f,  750.f, 1100.f,
   1600.f, 2300.f, 3300.f, 4700.f,
   6600.f, 9200.f, 12500.f, 16000.f
};

// =======================
// Frequency-dependent damping
// =======================
static const float baseDamping[STAGES] =
{
    7.4f, 7.7f, 8.1f, 8.5f,
    8.8f, 9.0f, 9.1f, 9.2f,
    9.3f, 9.4f, 9.5f, 9.6f,
    9.7f, 9.8f, 9.8f, 9.8f
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
    initializePin(pinDrift_);
    initializePin(pinBypass_);
}

// =======================
// open()
// =======================
int32_t RwSAllPass::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.f)
        sampleRate_ = 44100.f;

    updateCoefficients();

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
    updateCoefficients();

    // CPU save mode
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
// Update coefficients
// =======================
void RwSAllPass::updateCoefficients()
{
    float depth = std::clamp(pinDepth_.getValue(), 0.f, 1.f);

    for (int i = 0; i < STAGES; ++i)
    {
        float freqL = baseFreqs[i] * 0.994f;
        float freqR = baseFreqs[i] * 1.006f;

        float damp = baseDamping[i] + depth * 0.5f;
        damp = std::clamp(damp, 7.2f, 9.95f);

        float omegaL = 2.f * float(M_PI) * freqL / sampleRate_;
        float omegaR = 2.f * float(M_PI) * freqR / sampleRate_;

        apL_[i].a = (sinf(omegaL) - damp) / (sinf(omegaL) + damp);
        apR_[i].a = (sinf(omegaR) - damp) / (sinf(omegaR) + damp);

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

    if (pinInL_.isStreaming() && pinInR_.isStreaming())
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

    if (!inL || !inR || !outL || !outR)
        return;

    bool bypass = pinBypass_.getValue();

    for (int s = 0; s < sampleFrames; ++s)
    {
        if (bypass)
        {
            outL[s] = inL[s];
            outR[s] = inR[s];
            continue;
        }

        float l = inL[s];
        float r = inR[s];

        for (int i = 0; i < STAGES; ++i)
        {
            l = apL_[i].process(l);
            r = apR_[i].process(r);
        }

        outL[s] = l;
        outR[s] = r;
    }

    if (!pinInL_.isStreaming() || !pinInR_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOutL_.setStreaming(false);
        pinOutR_.setStreaming(false);
    }
}

// =======================
// Classic SynthEdit registration
// =======================
namespace
{
    auto r = Register<RwSAllPass>::withId(L"RwSAllPass");
    }
