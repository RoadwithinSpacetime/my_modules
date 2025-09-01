#pragma once
#include "mp_sdk_audio.h"

class CyclePeakLookahead : public MpBase
{
public:
    CyclePeakLookahead(IMpUnknown* host);

    void onSetPins() override;

private:
    // Pins (matching your XML)
    AudioInPin pinInput;
    AudioOutPin pinOutput;
    FloatOutPin pinPeak;

    FloatInPin pinLookahead;
    FloatInPin pinHysteresis;
    IntInPin pinAbsoluteMode;
};
