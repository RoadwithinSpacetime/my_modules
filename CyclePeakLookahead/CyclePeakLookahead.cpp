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

    // set default values
    pinLookaheadMs_.setValue(5.0f);
    pinHysteresis_.setValue(0.001f);
    pinAbsMode_.setValue(0);
    pinPeak_.setValue(0.0f);

    // safe initial state
    sampleRate_ = 44100.0;
    maxLookaheadSamples_ = static_cast<size_t>(0.03 * sampleRate_);
    lookaheadSamples_ = 1;        // avoid divide/mod by 0
    delay_.assign(maxLookaheadSamples_ + 256, 0.0f);
    delayWrite_ = 0;
    delayRead_ = 0;
    cyclePeak_ = 0.0f;
    lastSample_ = 0.0f;
}


int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();
    maxLookaheadSamples_ = static_cast<size_t>(std::ceil(0.03 * sampleRate_));

    // ensure valid size
    assert(maxLookaheadSamples_ > 0);

    delay_.assign(maxLookaheadSamples_ + 256, 0.0f);
    assert(!delay_.empty());

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
    assert(sampleRate_ > 0.0);

    double samples = pinLookaheadMs_.getValue() * 0.001 * sampleRate_;
    if (samples < 0.0)
        samples = 0.0;

    lookaheadSamples_ = static_cast<size_t>(samples);
    if (lookaheadSamples_ > maxLookaheadSamples_)
        lookaheadSamples_ = maxLookaheadSamples_;

    assert(lookaheadSamples_ <= delay_.size());
}

void CyclePeakLookahead::subProcess(int bufferOffset, int sampleFrames)
{
    assert(bufferOffset >= 0 && sampleFrames > 0);
    assert(delay_.size() > 0);

    float* in = pinIn_.getBuffer() + bufferOffset;
    float* out = pinOut_.getBuffer() + bufferOffset;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        float xAbs = absMode_ ? std::fabs(x) : x;

        // --- Delay line (lookahead) ---
        assert(delayWrite_ < delay_.size());
        delay_[delayWrite_] = x;
        delayWrite_ = (delayWrite_ + 1) % delay_.size();

        assert(delayRead_ < delay_.size());
        float y = delay_[delayRead_];
        delayRead_ = (delayRead_ + 1) % delay_.size();

        // --- Cycle peak detection ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
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
