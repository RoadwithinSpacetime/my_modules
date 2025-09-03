#include "CyclePeakLookahead.h"
#include <cmath>

using namespace gmpi;

CyclePeakLookahead::CyclePeakLookahead()
    {
    // Safe to initialize pins in constructor
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
}

int32_t CyclePeakLookahead::open()
{
    // Reset internal state
    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;

    // Set default output values
    pinPeak_.setValue(0.0f);

    // Register the processing callback
    setSubProcess(&CyclePeakLookahead::subProcess);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // Called when control pins change
    // No control pins in this simple version yet
}

void CyclePeakLookahead::subProcess(int bufferOffset, int sampleFrames)
{
    float* in = getBuffer(pinIn_) + bufferOffset;
    float* out = getBuffer(pinOut_) + bufferOffset;

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


// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
