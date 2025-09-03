#include "CyclePeakLookahead.h"

#undef max

CyclePeakLookahead::CyclePeakLookahead()
    : sampleRate_(44100.0)
    , maxLookaheadSamples_(0)
    , lookaheadSamples_(1)
    , delayWrite_(0)
    , delayRead_(0)
    , cyclePeak_(0.0f)
    , lastSample_(0.0f)
    , hysteresis_(0.001f)
    , absMode_(false)
{
    // Register pins
    initializePin(pinIn);
    initializePin(pinOut);

    initializePin(pinPeak);
    initializePin(pinLookaheadMs);
    initializePin(pinHysteresis);
    initializePin(pinAbsMode);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();
    maxLookaheadSamples_ = static_cast<size_t>(std::ceil(0.03 * sampleRate_));
    if (maxLookaheadSamples_ == 0)
        maxLookaheadSamples_ = 1;

    delay_.assign(maxLookaheadSamples_ + 256, 0.0f);

    // Initialize defaults (safe in SE 1.4 here, not in constructor)
    pinLookaheadMs = 5.0f;
    pinHysteresis = 0.001f;
    pinAbsMode = 0;
    pinPeak = 0.0f;

    updateLookahead();
    delayWrite_ = 0;
    delayRead_ = (delayWrite_ + delay_.size() - lookaheadSamples_) % delay_.size();

    setSubProcess(&CyclePeakLookahead::subProcess);
    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    hysteresis_ = std::max(0.0f, static_cast<float>(pinHysteresis.getValue()));
    absMode_ = (static_cast<int>(pinAbsMode.getValue()) != 0);

    updateLookahead();
}

void CyclePeakLookahead::updateLookahead()
{
    double samples = pinLookaheadMs.getValue() * 0.001 * sampleRate_;
    if (samples < 0.0)
        samples = 0.0;

    lookaheadSamples_ = static_cast<size_t>(samples);
    if (lookaheadSamples_ > maxLookaheadSamples_)
        lookaheadSamples_ = maxLookaheadSamples_;

    if (!delay_.empty())
        delayRead_ = (delayWrite_ + delay_.size() - lookaheadSamples_) % delay_.size();
    else
        delayRead_ = 0;
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    auto in = getBuffer(pinIn);
    auto out = getBuffer(pinOut);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        float xAbs = absMode_ ? std::fabs(x) : x;

        // --- Delay ---
        float y = x;
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
            pinPeak = cyclePeak_;
            cyclePeak_ = 0.0f;
        }

        if (xAbs > cyclePeak_ + hysteresis_)
            cyclePeak_ = xAbs;

        lastSample_ = x;

        // --- Output ---
        out[s] = y;
    }
}

// Registration
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
