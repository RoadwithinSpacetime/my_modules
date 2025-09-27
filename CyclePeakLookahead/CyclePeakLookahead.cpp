#include "CyclePeakLookahead.h"
#include <cstring>
#include <algorithm>
#undef max
#undef min

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
    , prevPeakSmoothIn_(0.0f)
    , nextPeakSmoothIn_(0.0f)
    , rampSamplesTotalIn_(1)
    , rampSamplesRemainingIn_(0)
    , prevPeakSmoothDelay_(0.0f)
    , nextPeakSmoothDelay_(0.0f)
    , rampSamplesTotalDelay_(1)
    , rampSamplesRemainingDelay_(0)
    , sampleRate_(0.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinMaxInputCycle_);
    initializePin(pinMaxDelayedCycle_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
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

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);  // ~30 ms
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
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
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    float* maxIn = getBuffer(pinMaxInputCycle_);
    float* maxDl = getBuffer(pinMaxDelayedCycle_);
    if (out)   memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut) memset(cvOut, 0, sampleFrames * sizeof(float));
    if (maxIn) memset(maxIn, 0, sampleFrames * sizeof(float));
    if (maxDl) memset(maxDl, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    float* maxIn = getBuffer(pinMaxInputCycle_);
    float* maxDel = getBuffer(pinMaxDelayedCycle_);
    if (!in || !out || !cvOut || !maxIn || !maxDel) return;

    float threshold = pinThreshold_ * 0.1f; // 0–1 to 0–10 V
    float ratio = pinRatio_;
    ratio = (std::max)(1.0f, (std::min)(20.0f, ratio));

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Zero-cross detection (start of a new input cycle) ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;

                // Compressor CV calculation
                float cvValue = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;
                    cvValue = compressedPeak / previousCyclePeak_;
                    cvValue = (std::max)(0.0f, (std::min)(1.0f, cvValue));
                }

                // backfill CV buffer for the finished cycle
                int fillCount = std::min(cycleLength, N);
                for (int i = 1; i <= fillCount; ++i)
                {
                    int idx = bufferWritePos_ - i;
                    idx %= N;
                    if (idx < 0) idx += N;
                    cvBuffer_[idx] = cvValue;
                }

                // --- Start smooth ramps for InputCycle ---
                prevPeakSmoothIn_ = nextPeakSmoothIn_;
                nextPeakSmoothIn_ = previousCyclePeak_;
                rampSamplesTotalIn_ = std::max(1, cycleLength);
                rampSamplesRemainingIn_ = rampSamplesTotalIn_;

                // --- Start smooth ramps for DelayedCycle ---
                prevPeakSmoothDelay_ = nextPeakSmoothDelay_;
                nextPeakSmoothDelay_ = previousCyclePeak_; // start with same snapshot
                rampSamplesTotalDelay_ = std::max(1, cycleLength);
                rampSamplesRemainingDelay_ = rampSamplesTotalDelay_;

                cyclePeak_ = 0.0f;
                samplesSinceCycleStart_ = 0;
            }
        }

        // --- Peak detection within current input cycle ---
#ifdef FULL_WAVE_PEAK
        float valueForPeak = std::fabs(x);
#else
        float valueForPeak = x;
#endif
        if (valueForPeak > cyclePeak_)
            cyclePeak_ = valueForPeak;

        lastSample_ = x;

        // --- Write to lookahead buffer (for audio delay) ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- Smooth interpolation for InputCycle peak ---
        float smoothedIn = nextPeakSmoothIn_;
        if (rampSamplesRemainingIn_ > 0)
        {
            float t = 1.0f - (float)rampSamplesRemainingIn_ / (float)rampSamplesTotalIn_;
            smoothedIn = prevPeakSmoothIn_ + t * (nextPeakSmoothIn_ - prevPeakSmoothIn_);
            --rampSamplesRemainingIn_;
        }

        // --- Smooth interpolation for DelayedCycle peak ---
        float smoothedDelay = nextPeakSmoothDelay_;
        if (rampSamplesRemainingDelay_ > 0)
        {
            float t = 1.0f - (float)rampSamplesRemainingDelay_ / (float)rampSamplesTotalDelay_;
            smoothedDelay = prevPeakSmoothDelay_ + t * (nextPeakSmoothDelay_ - prevPeakSmoothDelay_);
            --rampSamplesRemainingDelay_;
        }

        // --- Output audio and CVs ---
        int readPos = (bufferWritePos_ + 1) % N;
        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = cvBuffer_[readPos];
        maxIn[s] = smoothedIn;
        maxDel[s] = smoothedDelay;

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
