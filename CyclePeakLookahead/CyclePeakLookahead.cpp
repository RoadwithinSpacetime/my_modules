#include "CyclePeakLookahead.h"
#include <cstring>   // memset
#include <algorithm> // std::max, std::min
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
    , lastDelayedSample_(0.0f)
    , delayedCyclePeak_(0.0f)
    , delayedCyclePeakHold_(0.0f)
    , delayedSamplesSinceCycleStart_(0)
    , sampleRate_(0.0)
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

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_);
    minCycleGuard_ = lastPositiveWidth_ / 4;

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_); // ~30 ms
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
    float* maxInOut = getBuffer(pinMaxInputCycle_);
    float* maxDelayedOut = getBuffer(pinMaxDelayedCycle_);
    if (out)           memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut)         memset(cvOut, 0, sampleFrames * sizeof(float));
    if (maxInOut)      memset(maxInOut, 0, sampleFrames * sizeof(float));
    if (maxDelayedOut) memset(maxDelayedOut, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    float* maxInOut = getBuffer(pinMaxInputCycle_);
    float* maxDelayedOut = getBuffer(pinMaxDelayedCycle_);
    if (!in || !out || !cvOut || !maxInOut || !maxDelayedOut) return;

    float threshold = pinThreshold_ * 0.1f; // 0–1 to 0–10 V
    float ratio = pinRatio_;
    ratio = (std::max)(1.0f, (std::min)(20.0f, ratio));

    const int N = lookaheadSamples_;
    float currentInputPeak = 0.0f;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // ----- Input cycle zero-cross -----
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                float cvValue = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;
                    cvValue = compressedPeak / previousCyclePeak_;
                    cvValue = (std::max)(0.0f, (std::min)(1.0f, cvValue));
                }

                // optional quantization
                if (quantStep_ > 0.0f)
                {
                    if (useCeil_)
                        cvValue = std::ceil(cvValue / quantStep_) * quantStep_;
                    if (useFloor_)
                        cvValue = std::floor(cvValue / quantStep_) * quantStep_;
                    cvValue = (std::max)(0.0f, (std::min)(1.0f, cvValue));
                }

                // backfill CV buffer
                int fillCount = cycleLength;
                if (fillCount > N) fillCount = N;
                for (int i = 1; i <= fillCount; ++i)
                {
                    int idx = bufferWritePos_ - i;
                    idx %= N;
                    if (idx < 0) idx += N;
                    cvBuffer_[idx] = cvValue;
                }

                samplesSinceCycleStart_ = 0;
            }
        }

        // track input peak
#ifdef FULL_WAVE_PEAK
        float valueForPeak = std::fabs(x);
#else
        float valueForPeak = x;
#endif
        if (valueForPeak > cyclePeak_)
            cyclePeak_ = valueForPeak;
        if (valueForPeak > currentInputPeak)
            currentInputPeak = valueForPeak;

        lastSample_ = x;

        // --- Delay buffer write ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- Read delayed sample ---
        int readPos = (bufferWritePos_ + 1) % N;
        float delayedSample = lookaheadBuffer_[readPos];
        out[s] = delayedSample;
        cvOut[s] = cvBuffer_[readPos];

        // --- Delayed cycle tracking ---
        delayedSamplesSinceCycleStart_++;
        float delayedAbs = std::fabs(delayedSample);
        if (delayedAbs > delayedCyclePeak_)
            delayedCyclePeak_ = delayedAbs;

        // detect zero-cross of delayed signal
        if (lastDelayedSample_ <= 0.0f && delayedSample > 0.0f)
        {
            // hold previous peak for output, reset for next cycle
            delayedCyclePeakHold_ = delayedCyclePeak_;
            delayedCyclePeak_ = 0.0f;
            delayedSamplesSinceCycleStart_ = 0;
        }
        lastDelayedSample_ = delayedSample;

        // --- Output values ---
        maxInOut[s] = currentInputPeak;
        maxDelayedOut[s] = delayedCyclePeakHold_;

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
