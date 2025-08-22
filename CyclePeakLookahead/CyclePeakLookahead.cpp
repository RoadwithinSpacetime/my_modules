#include "mp_sdk_audio.h"
#include "CyclePeakLookahead.h"

REGISTER_PLUGIN(CyclePeakLookahead, L"CyclePeakLookahead");

CyclePeakLookahead::CyclePeakLookahead(IMpUnknown* host)
    : MpBase(host)
    , pinIn_(this, 0)
    , pinOut_(this, 1)
    , pinPeak_(this, 2)
    , pinLookaheadMs_(this, 3)
    , pinHysteresis_(this, 4)
    , pinAbsMode_(this, 5)
{
    pinPeak_ = 0.0f;
}

void CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate(); // DAW-provided sample rate
    maxLookaheadSamples_ = static_cast<int>(ceil(0.030 * sampleRate_));
    delay_.assign(maxLookaheadSamples_ + 256, 0.0f);
    delayWrite_ = 0;
    updateLookahead();
    delayRead_ = (delayWrite_ + delay_.size() - (size_t)lookaheadSamples_) % delay_.size();
    SET_PROCESS(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::updateLookahead()
{
    float ms = pinLookaheadMs_;
    if (ms < 0.f) ms = 0.f;
    if (ms > 30.f) ms = 30.f;

    lookaheadSamples_ = static_cast<int>(round(ms * 0.001 * sampleRate_));
    hysteresis_ = std::max(0.0f, pinHysteresis_.getValue());
    absMode_ = (pinAbsMode_.getValue() != 0);

    if (!delay_.empty())
    {
        delayRead_ = (delayWrite_ + delay_.size() - (size_t)lookaheadSamples_) % delay_.size();
    }
}

bool CyclePeakLookahead::zeroCrossingNegToPos(float prev, float now, float hysteresis) const
{
    return (prev <= -hysteresis && now >= hysteresis);
}

void CyclePeakLookahead::onSetPins()
{
    if (pinLookaheadMs_.isUpdated() || pinHysteresis_.isUpdated() || pinAbsMode_.isUpdated())
    {
        updateLookahead();
    }
    SET_PROCESS(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::subProcess(int bufferOffset, int sampleFrames)
{
    float* __restrict in = bufferOffset + pinIn_.getBuffer();
    float* __restrict out = bufferOffset + pinOut_.getBuffer();

    float last = haveLast_ ? lastSample_ : (sampleFrames > 0 ? in[0] : 0.0f);

    for (int i = 0; i < sampleFrames; ++i)
    {
        const float x = in[i];
        const float ax = absMode_ ? fabs(x) : x;

        if (!haveLast_)
        {
            haveLast_ = true;
            currentCycleStartPos_ = inputSamplePos_;
            currentCycleMax_ = ax;
        }
        else
        {
            if (ax > currentCycleMax_) currentCycleMax_ = ax;
        }

        if (zeroCrossingNegToPos(last, x, hysteresis_))
        {
            cycles_.push_back({ currentCycleStartPos_, currentCycleMax_ });
            currentCycleStartPos_ = inputSamplePos_;
            currentCycleMax_ = ax;
        }

        ++inputSamplePos_;
        last = x;

        delay_[delayWrite_] = x;
        delayWrite_ = (delayWrite_ + 1) % delay_.size();
        float y = delay_[delayRead_];
        delayRead_ = (delayRead_ + 1) % delay_.size();

        while (!cycles_.empty() && (cycles_.front().startPos + lookaheadSamples_) <= outputSamplePos_)
        {
            pinPeak_ = cycles_.front().peak;
            cycles_.pop_front();
        }

        out[i] = y;
        ++outputSamplePos_;
    }

    lastSample_ = last;
}
