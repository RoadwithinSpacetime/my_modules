#include "CyclePeakLookahead.h"
#include <cmath>

using namespace gmpi;

CyclePeakLookahead::CyclePeakLookahead()
{
    // Initialize pins
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
}

int32_t CyclePeakLookahead::open()
{
    // Reset internal state
    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;

    // Default output
    pinPeak_.setValue(0.0f);

    // Register audio processing callback
    setSubProcess(&CyclePeakLookahead::subProcess);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // No control pins yet
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // --- Cycle peak detection ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            pinPeak_.setValue(cyclePeak_);
            cyclePeak_ = 0.0f;
        }

        if (x > cyclePeak_)
            cyclePeak_ = x;

        lastSample_ = x;

        // --- Pass-through ---
        out[s] = x;
    }
}

// Register plugin with SE1.4
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
