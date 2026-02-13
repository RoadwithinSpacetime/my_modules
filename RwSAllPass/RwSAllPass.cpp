#include "RwSAllPass.h"
#include <algorithm>
#include <cstring>

#undef min
#undef max

// =======================
// Fairchild-style frequency layout
// =======================
static const float baseFreqs[STAGES] =
{
     35.f,   55.f,   85.f,  130.f,
    200.f,  320.f,  500.f,  750.f,
   1100.f, 1600.f, 2300.f, 3300.f,
   4700.f, 6500.f, 9000.f, 13000.f
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

    // Ensure immediate response
    setSubProcess(&RwSAllPass::subProcess);
    pinOutL_.setStreaming(true);
    pinOutR_.setStreaming(true);
}

// =======================
// Fairchild-shaped coefficients
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

        // -----------------------
        // Per-band weighting
        // -----------------------
        float stageWeight;
        if (i < 4)        stageWeight = 1.00f; // subs
        else if (i < 7)   stageWeight = 0.85f; // low mids
        else if (i < 10)  stageWeight = 0.65f; // mids
        else if (i < 13)  stageWeight = 0.40f; // upper mids
        else              stageWeight = 0.25f; // highs

        // -----------------------
        // Depth scaling (gentle)
        // -----------------------
        g *= stageWeight;
        g *= (0.18f + 1.4f * depthShape);

        // -----------------------
        // HF taper (iron behavior)
        // -----------------------
        float hfTaper = 1.0f - 0.6f *
            std::clamp((freq - 6000.f) / 10000.f, 0.f, 1.f);
        g *= hfTaper;

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

    bool bypass = pinBypass_.getValue();
    float drift = std::clamp(pinDrift_.getValue(), 0.0f, 1.0f);
    float driftAmt = drift * 0.002f;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float l = inL ? inL[s] : 0.0f;
        float r = inR ? inR[s] : l;

        if (!bypass)
        {
            for (int i = 0; i < STAGES; ++i)
            {
                float mod = 1.0f + driftAmt * sinf(driftPhase_[i]);
                driftPhase_[i] += 0.00004f * (i + 1);
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
