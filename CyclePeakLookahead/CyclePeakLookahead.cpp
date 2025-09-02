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

    int64_t temp = static_cast<int64_t>(pinLookaheadMs_.getValue() * 0.001f * sampleRate_);

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
        float xAbs = absMode_ ? std::fabs(x) : x;

        // --- Delay line (lookahead) ---
        delay_[delayWrite_] = x;
        delayWrite_ = (delayWrite_ + 1) % delay_.size();

        float y = delay_[delayRead_];
        delayRead_ = (delayRead_ + 1) % delay_.size();

        // --- Cycle peak detection ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            // New cycle starts: output the peak of the last cycle
            pinPeak_.setValue(cyclePeak_);
            cyclePeak_ = 0.0f;
        }

        if (xAbs > cyclePeak_)
            cyclePeak_ = xAbs;

        lastSample_ = x;

        // Output delayed signal
        out[s] = y;
    }
}
