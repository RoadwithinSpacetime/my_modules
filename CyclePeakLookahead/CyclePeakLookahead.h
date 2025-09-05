#pragma once
#include "mp_sdk_audio.h"

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
public:
    CyclePeakLookahead();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames);

private:
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinPeak_;

    float lastSample_;
    float cyclePeak_;
};
