#include "CyclePeakLookahead.h"

using namespace gmpi;

CyclePeakLookahead::CyclePeakLookahead()
    : lastSample_(0.0f)
    , cyclePeak_(0.0f)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
}

int32_t CyclePeakLookahead::open()
{
    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;


    setSubProcess(&CyclePeakLookahead::subProcess);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // Streaming must be enabled for audio-rate outputs
    pinOut_.setStreaming(true);
    pinPeak_.setStreaming(true);

    setSubProcess(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* peakOut = getBuffer(pinPeak_);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // Detect positive zero-crossing
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            cyclePeak_ = 0.0f; // Reset for next cycle
        }

        // Track maximum of current cycle
        if (x > cyclePeak_)
            cyclePeak_ = x;

        lastSample_ = x;

        // Pass-through audio
        out[s] = x;

        // Output instantaneous cycle peak (audio-rate)
        peakOut[s] = cyclePeak_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
