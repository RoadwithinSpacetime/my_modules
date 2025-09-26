#include "CyclePeakLookahead.h"
#include <algorithm>
#include <cstring>
#undef max
#undef min  // protect against Windows macros

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    , maxFundamentalHz_(4000.0f)      // default: ignore cycles >4 kHz
    , minCycleSamples_(0)
    , sampleRate_(0.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
    initializePin(pinAttack_);
    initializePin(pinRelease_);
    initializePin(pinMaxFundamental_); // NEW
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;
    previousCyclePeak_ = 0.0f;
    samplesSinceCycleStart_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // default 10 ms
    minCycleGuard_ = lastPositiveWidth_ / 4;

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    bufferWritePos_ = 0;

    // compute minimum allowed cycle length from default maxFundamentalHz_
    minCycleSamples_ = std::max(1, static_cast<int>(sampleRate_ / maxFundamentalHz_));

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // update maximum fundamental if user changes it
    float newMaxFund = static_cast<float>(pinMaxFundamental_.getValue());
    if (newMaxFund > 0.0f)
    {
        maxFundamentalHz_ = newMaxFund;
        minCycleSamples_ = std::max(1, static_cast<int>(sampleRate_ / maxFundamentalHz_));
    }

    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    setSubProcess(&CyclePeakLookahead::subProcess);
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
    if (out)   std::memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut) std::memset(cvOut, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    if (!in || !out || !cvOut) return;

    float threshold = pinThreshold_ * 0.1f; // map 0–1 -> 0–10 V
    float ratio = std::clamp(static_cast<float>(pinRatio_), 1.0f, 20.0f);

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // rising zero-crossing
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // --- NEW: only process if cycle is long enough ---
                if (cycleLength >= minCycleSamples_)
                {
                    float cvValue = 1.0f;
                    if (previousCyclePeak_ > 0.0f)
                    {
                        float over = previousCyclePeak_ - threshold;
                        if (over < 0.0f) over = 0.0f;
                        float compressedPeak = threshold + over / ratio;
                        cvValue = compressedPeak / previousCyclePeak_;
                        cvValue = std::clamp(cvValue, 0.0f, 1.0f);
                    }

                    int fillCount = std::min(cycleLength, N);
                    for (int i = 1; i <= fillCount; ++i)
                    {
                        int idx = bufferWritePos_ - i;
                        idx %= N;
                        if (idx < 0) idx += N;
                        cvBuffer_[idx] = cvValue;
                    }
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

        lookaheadBuffer_[bufferWritePos_] = x;

        int readPos = (bufferWritePos_ + 1) % N;
        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = cvBuffer_[readPos];

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
