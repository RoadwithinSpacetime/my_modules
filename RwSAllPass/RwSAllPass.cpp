#include "RwSAllPass.h"
#include <cmath>

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
RwSAllPass::RwSAllPass(IMpUnknown* host)
    : MpBase(host)
{
    initializePin(pinInL);
    initializePin(pinInR);
    initializePin(pinOutL);
    initializePin(pinOutR);

    initializePin(pinDepth);
    initializePin(pinDrift);
    initializePin(pinBypass);
}

// =======================
// Pin change callback
// =======================
void RwSAllPass::onSetPins()
{
    updateCoefficients();
}

// =======================
// Update all-pass coefficients
// =======================
void RwSAllPass::updateCoefficients()
{
    float depth = pinDepth.getValue();
    float sr = getSampleRate();

    if (sr <= 0.f)
        return;

    for (int i = 0; i < STAGES; ++i)
    {
        float freqL = baseFreqs[i] * 0.994f;
        float freqR = baseFreqs[i] * 1.006f;

        float damp = baseDamping[i] + depth * 0.5f;

        if (damp < 7.2f)  damp = 7.2f;
        if (damp > 9.95f) damp = 9.95f;

        float omegaL = 2.f * 3.14159265359f * freqL / sr;
        float omegaR = 2.f * 3.14159265359f * freqR / sr;

        apL[i].a = (sinf(omegaL) - damp) / (sinf(omegaL) + damp);
        apR[i].a = (sinf(omegaR) - damp) / (sinf(omegaR) + damp);
    }
}

// =======================
// Audio processing
// =======================
void RwSAllPass::process(int sampleFrames)
{
    float* inL = pinInL.getBuffer();
    float* inR = pinInR.getBuffer();
    float* outL = pinOutL.getBuffer();
    float* outR = pinOutR.getBuffer();

    bool bypass = pinBypass.getValue() > 0.5f;

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
            l = apL[i].process(l);
            r = apR[i].process(r);
        }

        outL[s] = l;
        outR[s] = r;
    }
}
