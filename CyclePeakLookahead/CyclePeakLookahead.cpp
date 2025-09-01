#include "CyclePeakLookahead.h"

REGISTER_PLUGIN(CyclePeakLookahead, L"CyclePeakLookahead");

CyclePeakLookahead::CyclePeakLookahead(IMpUnknown* host)
    : MpBase(host)
    , pinInput(L"Input")
    , pinOutput(L"Output")
    , pinPeak(L"Peak")
    , pinLookahead(L"Lookahead (ms)")
    , pinHysteresis(L"Hysteresis")
    , pinAbsoluteMode(L"Absolute Mode")
{
    initializePin(pinInput);
    initializePin(pinOutput);
    initializePin(pinPeak);
    initializePin(pinLookahead);
    initializePin(pinHysteresis);
    initializePin(pinAbsoluteMode);
}

void CyclePeakLookahead::onSetPins()
{
    // Audio: copy input directly to output
    if (pinInput.isStreaming())
    {
        int sampleFrames = getBlockSize();
        float* in = pinInput.getBuffer();
        float* out = pinOutput.getBuffer();

        if (in && out)
        {
            for (int i = 0; i < sampleFrames; ++i)
            {
                out[i] = in[i];
            }
        }
    }

    // Peak output fixed to 0.0 for now
    pinPeak = 0.0f;
}
