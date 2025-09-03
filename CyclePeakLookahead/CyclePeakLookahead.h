#pragma once
#include "../se_sdk3/mp_sdk_audio.h"

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
public:
    CyclePeakLookahead();

    int32_t open() override;
    void subProcess(int sampleFrames);
    void onSetPins() override;

private:
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;

    float cyclePeak_;
    float lastSample_;
};
