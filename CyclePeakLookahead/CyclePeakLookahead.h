#pragma once
#include "../se_sdk3/mp_sdk_audio.h"

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
    // Pins
    AudioInPin  pinIn;
    AudioOutPin pinOut;

public:
    CyclePeakLookahead();

    void onSetPins() override;
    void subProcess(int sampleFrames);
    int32_t open() override;
};
