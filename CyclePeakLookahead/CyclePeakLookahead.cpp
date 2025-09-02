#include "CyclePeakLookahead.h"

#undef max

REGISTER_PLUGIN(CyclePeakLookahead, L"CyclePeakLookahead");

CyclePeakLookahead::CyclePeakLookahead(IMpUnknown* host)
    : MpBase(host),
    sampleRate_(44100.0),
    maxLookaheadSamples_(0),
    lookaheadSamples_(1),
    delayWrite_(0),
    delayRead_(0),
    cyclePeak_(0.0f),
    lastSample_(0.0f),
    hysteresis_(0.001f),
    absMode_(false)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
    initializePin(pinLookaheadMs_);
    initializePin(pinHysteresis_);
    initializePin(pinAbsMode_);

    // default pin values
    pinLookaheadMs_.setValue(5.0f);
    pinHysteresis_.setValue(0.001f);
    pinAbsMode_.setValue(0);
    pinPeak_.setValue(0.0f);

    // initialize delay buffer
    maxLookaheadSamples_ = static_cast<size_t>(0.03 * sampleRate_);
    delay_.assign(maxLookaheadSamples_ + 256, 0.0f);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();
    maxLookaheadSamples_ = static_cast<size_t>(std::ceil(0.03 * sampleRate_));
    delay_.resize(maxLookaheadSamples_ + 256, 0.0f);

    updateLookahead();
    delayWrite_ = 0;
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
    double samples = pinLookaheadMs_.getValue() * 0.001 * sampleRate_;
    if (samples < 0.0) samples = 0.0;

    lookaheadSamples_ = static_cast<size_t>(samples);
    if (lookaheadSamples_ > maxLookaheadSamples_)
        lookaheadSamples_ = maxLookaheadSamples_;

    // safe delayRead_ initialization
    if (!delay_.empty())
        delayRead_ = (delayWrite_ + delay_.size() - lookaheadSamples_) % delay_.size();
    else
        delayRead_ = 0;
}

void CyclePeakLookahead::subProcess(int bufferOffset, int sampleFrames)
{
    float* in = pinIn_.getBuffer() + bufferOffset;
    float* out = pinOut_.getBuffer() + bufferOffset;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        float xAbs = absMode_ ? std::fabs(x) : x;

        // --- Delay / lookahead ---
        float y = x;  // fallback if delay empty
        if (!delay_.empty())
        {
            delay_[delayWrite_] = x;
            delayWrite_ = (delayWrite_ + 1) % delay_.size();

            y = delay_[delayRead_];
            delayRead_ = (delayRead_ + 1) % delay_.size();
        }

        // --- Cycle peak detection ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            pinPeak_.setValue(cyclePeak_);
            cyclePeak_ = 0.0f;
        }

        if (xAbs > cyclePeak_ + hysteresis_)
            cyclePeak_ = xAbs;

        lastSample_ = x;

        // --- Output ---
        out[s] = y;
    }
}
