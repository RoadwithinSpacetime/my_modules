#include "CyclePeakLookahead.h"

using namespace gmpi;

CyclePeakLookahead::CyclePeakLookahead()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
}

int32_t CyclePeakLookahead::open()
{
    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;
    pinPeak_.setValue(0.0f);

    setSubProcess(&CyclePeakLookahead::subProcess);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // Always streaming
    pinOut_.setStreaming(true);
    setSubProcess(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            pinPeak_.setValue(cyclePeak_);
            cyclePeak_ = 0.0f;
        }

        if (x > cyclePeak_)
            cyclePeak_ = x;

        lastSample_ = x;
        out[s] = x;
    }
}

namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
