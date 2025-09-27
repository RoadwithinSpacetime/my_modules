#include "CyclePeakLookahead.h"
#include <cstring>    // memset
#include <algorithm>  // std::clamp
#undef max
#undef min  // avoid Windows macros

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FULL_WAVE_PEAK


CyclePeakLookahead::CyclePeakLookahead()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinMaxInputCycle_);
    initializePin(pinMaxDelayedCycle_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
    initializePin(pinAttack_);
    initializePin(pinRelease_);
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

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);  // 30 ms
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    maxBuffer_.assign(lookaheadSamples_, 0.0f);
    bufferWritePos_ = 0;

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    pinMaxInputCycle_.setStreaming(true);
    pinMaxDelayedCycle_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    pinMaxInputCycle_.setStreaming(true);
    pinMaxDelayedCycle_.setStreaming(true);
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
        pinMaxInputCycle_.setStreaming(true);
        pinMaxDelayedCycle_.setStreaming(true);
        subProcess(sampleFrames);
        return;
    }

    float* out = getBuffer(pinOut_);
    float* cv = getBuffer(pinCV_);
    float* maxI = getBuffer(pinMaxInputCycle_);
    float* maxD = getBuffer(pinMaxDelayedCycle_);

    if (out)  memset(out, 0, sampleFrames * sizeof(float));
    if (cv)   memset(cv, 0, sampleFrames * sizeof(float));
    if (maxI) memset(maxI, 0, sampleFrames * sizeof(float));
    if (maxD) memset(maxD, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cv = getBuffer(pinCV_);
    float* maxI = getBuffer(pinMaxInputCycle_);
    float* maxD = getBuffer(pinMaxDelayedCycle_);
    if (!in || !out || !cv || !maxI || !maxD) return;

    float threshold = pinThreshold_ * 0.1f; // map 0–1 to 0–10 V
    float ratio = std::max(1.0f, std::min(20.0f, static_cast<float>(pinRatio_)));


    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Zero-cross detection ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                inputCyclePeakHold_ = previousCyclePeak_; // snapshot input peak

                // compute CV gain
                float cvValue = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;
                    cvValue = compressedPeak / previousCyclePeak_;
                    cvValue = std::clamp(cvValue, 0.0f, 1.0f);
                }

                // backfill buffers for this cycle
                int fillCount = cycleLength;
                if (fillCount > N) fillCount = N;
                for (int i = 1; i <= fillCount; ++i)
                {
                    int idx = bufferWritePos_ - i;
                    idx %= N;
                    if (idx < 0) idx += N;
                    cvBuffer_[idx] = cvValue;
                    maxBuffer_[idx] = inputCyclePeakHold_;
                }

                cyclePeak_ = 0.0f;
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

        // --- Delay readout ---
        lookaheadBuffer_[bufferWritePos_] = x;

        int readPos = (bufferWritePos_ + 1) % N;
        out[s] = lookaheadBuffer_[readPos];
        cv[s] = cvBuffer_[readPos];
        maxI[s] = inputCyclePeakHold_;      // live input cycle peak
        delayedCyclePeakHold_ = maxBuffer_[readPos];
        maxD[s] = delayedCyclePeakHold_;    // delayed cycle peak

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
