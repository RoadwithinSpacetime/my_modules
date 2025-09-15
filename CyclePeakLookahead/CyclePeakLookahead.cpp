#include "CyclePeakLookahead.h"
#include <algorithm> // std::max
#include <cstring>   // memset

#undef max
#undef min

#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
    : lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , previousCyclePeak_(0.0f)
    , bufferWritePos_(0)
    , lookaheadSamples_(0)
    , samplesSinceCycleStart_(0)
    , lastPositiveWidth_(0)
    , minCycleGuard_(0)
    , sampleRate_(0.0)
    , cvPending_(1.0f)
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

    // 30 ms lookahead buffer
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvDelayBuffer_.assign(lookaheadSamples_, 1.0f);
    bufferWritePos_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // default 10ms
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

    float threshold = pinThreshold_ * 0.1f; // map 0–1 -> 0–10 V
    float ratio = std::clamp((float)pinRatio_, 1.0f, 20.0f);

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Detect positive zero-crossing (input domain) ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                // store previous peak
                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // Compute next CV based on completed input cycle
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressed = threshold + over / ratio;

                    // Normalize to peak, range 0..1
                    cvPending_ = std::clamp(compressed / previousCyclePeak_, 0.0f, 1.0f);
                }
                else
                {
                    cvPending_ = 1.0f;
                }

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

        // --- Delay both audio and CV equally ---
        lookaheadBuffer_[bufferWritePos_] = x;
        cvDelayBuffer_[bufferWritePos_] = cvPending_;

        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = cvDelayBuffer_[readPos];

        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
