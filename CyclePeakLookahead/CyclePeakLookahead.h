#pragma once
#include "../se_sdk3/mp_sdk_audio.h"

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
public:
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    FloatOutPin pinPeak_;

private:
    float cyclePeak_ = 0.0f;
    float lastSample_ = 0.0f;

public:
    CyclePeakLookahead();
    void subProcess(int sampleFrames);
    void onSetPins() override;
};
