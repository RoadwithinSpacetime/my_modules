#include "CyclePeakLookahead.h"

CyclePeakLookahead::CyclePeakLookahead()
{
    initializePin(pinIn);
    initializePin(pinOut);
}

int32_t CyclePeakLookahead::open()
{
    // Always set the processing method.
    setSubProcess(&CyclePeakLookahead::subProcess);
    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // Pass audio through.
    pinOut.setStreaming(pinIn.isStreaming());
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    auto in = getBuffer(pinIn);
    auto out = getBuffer(pinOut);

    for (int s = 0; s < sampleFrames; ++s)
    {
        out[s] = in[s]; // x = y
    }
}

// Register the module
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
