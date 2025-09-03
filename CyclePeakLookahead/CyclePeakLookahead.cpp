#include "CyclePeakLookahead.h"
#include <cmath>

using namespace gmpi;

CyclePeakLookahead::CyclePeakLookahead()
{
    // Initialize pins safely in constructor
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
}

int32_t CyclePeakLookahead::open()
{
    // Initialize internal state
    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;

    // Default output
    pinPeak_.setValue(0.0f);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // Enable streaming and register processing callback
    if (pinIn_.isStreaming() || pinOut_.isStreaming())
    {
        pinOut_.setStreaming(true);
        setSubProcess(&CyclePeakLookahead::subProcess);
    }
    else
    {
        setSubProcess(nullptr); // stop processing if no audio
    }
}

void CyclePeakLookahead::subProcess(int bufferOffset, int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);

    if (!in || !out)
        return;

    in += bufferOffset;
    out += bufferOffset;

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
