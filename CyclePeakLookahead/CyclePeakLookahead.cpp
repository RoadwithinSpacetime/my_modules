#include "CyclePeakLookahead.h"

#undef max

REGISTER_PLUGIN(CyclePeakLookahead, L"CyclePeakLookahead");

CyclePeakLookahead::CyclePeakLookahead(IMpUnknown* host)
    : MpBase(host)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
    initializePin(pinLookaheadMs_);
    initializePin(pinHysteresis_);
    initializePin(pinAbsMode_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();
    maxLookaheadSamples_ = static_cast<int>(ceil(0.03f * sampleRate_));

    delay_.assign(maxLookaheadSamples_ + 256, 0.0f);
    delayWrite_ = 0;
    lookaheadSamples_ = 0;
    updateLookahead();
    delayRead_ = (delayWrite_ + delay_.size() - lookaheadSamples_) % delay_.size();

   setSubProcess(static_cast<SubProcess_ptr>(&CyclePeakLookahead::subProcess));

       return MpBase::open();
}

void CyclePeakLookahead::onSetPins()
{
    hysteresis_ = std::max(0.0f, static_cast<float>(pinHysteresis_.getValue()));
        absMode_ = pinAbsMode_.getValue() != 0;

    updateLookahead();
}

void CyclePeakLookahead::updateLookahead()
{
    sampleRate_ = getSampleRate();

    lookaheadSamples_ = static_cast<int>(pinLookaheadMs_.getValue() * 0.001f * sampleRate_);
    if (lookaheadSamples_ > maxLookaheadSamples_) lookaheadSamples_ = maxLookaheadSamples_;
    if (lookaheadSamples_ < 0) lookaheadSamples_ = 0;
}

void CyclePeakLookahead::subProcess(int bufferOffset, int sampleFrames)
{
    float* in = pinIn_.getBuffer() + bufferOffset;
    float* out = pinOut_.getBuffer() + bufferOffset;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        if (absMode_) x = std::fabs(x);

        delay_[delayWrite_] = x;
        delayWrite_ = (delayWrite_ + 1) % delay_.size();

        float y = delay_[delayRead_];
        delayRead_ = (delayRead_ + 1) % delay_.size();

        if (x > currentPeak_)
            currentPeak_ = x;
        else
        {
            currentPeak_ -= hysteresis_;
            if (currentPeak_ < 0.0f) currentPeak_ = 0.0f;
        }

        out[s] = y;
    }

    pinPeak_ = currentPeak_;
}
