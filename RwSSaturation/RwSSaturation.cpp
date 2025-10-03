#include "RwSSaturation.h"
#include <algorithm>

RwSSaturation::RwSSaturation()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
}

int32_t RwSSaturation::open()
{
    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    bool streaming = pinIn_.isStreaming();
    pinOut_.setStreaming(streaming);

    setSubProcess(&RwSSaturation::subProcess);
}

void RwSSaturation::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out) return;

    float drive = (std::max)(0.1f, pinDrive_.getValue()); // prevent zero
    float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float dry = in[s];
        float wet = saturate(dry * drive);

        // Mix dry/wet
        out[s] = dry * (1.0f - mix) + wet * mix;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
