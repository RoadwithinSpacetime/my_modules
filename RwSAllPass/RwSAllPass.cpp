#include "RwSAllPass.h"
#include <algorithm>
#include <cstring>

#undef min
#undef max

// =======================
// Frequency layout (analog-like)
// =======================
static const float baseFreqs[STAGES] =
{
     60.f,   90.f,  140.f,  220.f,
    340.f,  520.f,  780.f, 1150.f,
   1700.f, 2500.f, 3600.f, 5200.f,
   7400.f, 9800.f, 13000.f, 17000.f
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
// =======================
void RwSAllPass::onSetPins()
{
    updateCoefficients();

    // Ensure DSP reacts immediately
    setSubProcess(&RwSAllPass::subProcess);
    pinOutL_.setStreaming(true);
    pinOutR_.setStreaming(true);
}

// =======================
// Coefficients (stable + audible)
// =======================
void RwSAllPass::updateCoefficients()
{
    float depth = std::clamp(pinDepth_.getValue(), 0.0f, 1.0f);
    float depthShape = depth * depth;

    for (int i = 0; i < STAGES; ++i)
    {
        float freq = baseFreqs[i];
        float omega = 2.0f * float(M_PI) * freq / sampleRate_;
        float g = tanf(omega * 0.5f);

        // Depth scaling (audible but safe)
        g *= (0.15f + 3.0f * depthShape);

        float a = (1.0f - g) / (1.0f + g);

        apL_[i].a = a;
        apR_[i].a = a;

        apL_[i].reset();
        apR_[i].reset();
    }
}

// =======================
// Silent
// =======================
void RwSAllPass::subProcessSilent(int sampleFrames)
{
    float* outL = getBuffer(pinOutL_);
    float* outR = getBuffer(pinOutR_);

    if (outL) std::memset(outL, 0, sampleFrames * sizeof(float));
    if (outR) std::memset(outR, 0, sampleFrames * sizeof(float));

    if (pinInL_.isStreaming() || pinInR_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcess);
        pinOutL_.setStreaming(true);
        pinOutR_.setStreaming(true);
    }
}

// =======================
// Audio
// =======================
void RwSAllPass::subProcess(int sampleFrames)
{
    const float* inL = getBuffer(pinInL_);
    const float* inR = getBuffer(pinInR_);
    float* outL = getBuffer(pinOutL_);
    float* outR = getBuffer(pinOutR_);

    if (!outL || !outR)
        return;

    bool bypass = pinBypass_.getValue();
    float drift = std::clamp(pinDrift_.getValue(), 0.0f, 1.0f);
    float driftAmt = drift * 0.0025f;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float l = inL ? inL[s] : 0.0f;
        float r = inR ? inR[s] : l;

        if (!bypass)
        {
            for (int i = 0; i < STAGES; ++i)
            {
                float mod = 1.0f + driftAmt * sinf(driftPhase_[i]);
                driftPhase_[i] += 0.00005f * (i + 1);
                if (driftPhase_[i] > 2.f * float(M_PI))
                    driftPhase_[i] -= 2.f * float(M_PI);

                l = apL_[i].process(l * mod);
                r = apR_[i].process(r / mod);
            }
        }

        outL[s] = l;
        outR[s] = r;
    }

    if (!pinInL_.isStreaming() && !pinInR_.isStreaming())
    {
        setSubProcess(&RwSAllPass::subProcessSilent);
        pinOutL_.setStreaming(false);
        pinOutR_.setStreaming(false);
    }
}

// =======================
// Registration
// =======================
namespace
{
    auto r = Register<RwSAllPass>::withId(L"RwSAllPass");
}
