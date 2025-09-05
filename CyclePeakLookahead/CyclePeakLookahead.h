#pragma once
#include "mp_sdk_audio.h"

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
public:
    CyclePeakLookahead();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames); // SE1.4-compatible signature

private:
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;

    float lastSample_;
    float cyclePeak_;
};
