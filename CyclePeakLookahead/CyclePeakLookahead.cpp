#include "CyclePeakLookahead.h"
#include <cstring> // memset

#undef max
#undef min
#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
    : bufferWritePos_(0)
    , lookaheadSamples_(0)
    , lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , previousCyclePeak_(0.0f)
    , samplesSinceCycleStart_(0)
    , lastPositiveWidth_(0)
    , minCycleGuard_(0)
    , currentCv_(1.0f)
    , nextCvTarget_(1.0f)
    , promoteIndex_(-1)
    , sampleRate_(0.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    // 30 ms lookahead
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    bufferWritePos_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // default 10 ms
    minCycleGuard_ = lastPositiveWidth_ / 4;

    setSubProcess(&CyclePeakLookahead::subProcessSilent);
    pinOut_.setStreaming(false);
    pinCV_.setStreaming(false);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    if (pinIn_.isStreaming())
    {
        setSubProcess(&CyclePeakLookahead::subProcess);
        pinOut_.setStreaming(true);
        pinCV_.setStreaming(true);
    }
    else
    {
        setSubProcess(&CyclePeakLookahead::subProcessSilent);
        pinOut_.setStreaming(false);
        pinCV_.setStreaming(false);
    }
}

void CyclePeakLookahead::subProcessSilent(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    if (in && pinIn_.isStreaming())
    {
        setSubProcess(&CyclePeakLookahead::subProcess);
        pinOut_.setStreaming(true);
        pinCV_.setStreaming(true);
        subProcess(sampleFrames);
        return;
    }

    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    if (out) memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut) memset(cvOut, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);

    if (!in || !out || !cvOut)
        return;

    const float threshold = pinThreshold_ * 0.1f; // 0–10 V
    float ratio = pinRatio_;
    if (ratio < 1.0f) ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    const int bufferSize = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- detect zero crossing on INPUT ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // Compute next CV target
                float newTarget = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;
                    newTarget = compressedPeak / previousCyclePeak_;
                    if (newTarget < 0.0f) newTarget = 0.0f;
                    if (newTarget > 1.0f) newTarget = 1.0f;
                }

                nextCvTarget_ = newTarget;

                // Calculate promotion index in circular buffer
                // We want to update currentCv_ when the delayed audio reaches THIS crossing
                promoteIndex_ = (bufferWritePos_ + 1) % bufferSize;

                samplesSinceCycleStart_ = 0;
            }
        }

#ifdef FULL_WAVE_PEAK
        float valueForPeak = std::fabs(x);
#else
        float valueForPeak = x;
#endif
        if (valueForPeak > cyclePeak_)
            cyclePeak_ = valueForPeak;

        lastSample_ = x;

        // --- write to lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- read delayed sample ---
        int readPos = (bufferWritePos_ + 1) % bufferSize;

        // Promote CV when delayed read pointer hits scheduled index
        if (promoteIndex_ >= 0 && readPos == promoteIndex_)
        {
            currentCv_ = nextCvTarget_;
            promoteIndex_ = -1; // clear until next detection
        }

        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = currentCv_;

        bufferWritePos_ = (bufferWritePos_ + 1) % bufferSize;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
