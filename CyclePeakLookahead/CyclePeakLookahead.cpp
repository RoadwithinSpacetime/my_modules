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
    , sampleRate_(0.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinMaxInputCycle_);   // NEW
    initializePin(pinMaxDelayedCycle_); // NEW
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

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    bufferWritePos_ = 0;

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    pinMaxInputCycle_.setStreaming(true);   // NEW
    pinMaxDelayedCycle_.setStreaming(true); // NEW

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    pinMaxInputCycle_.setStreaming(true);   // NEW
    pinMaxDelayedCycle_.setStreaming(true); // NEW
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

    float threshold = pinThreshold_ * 0.1f;
    float ratio = pinRatio_;
    ratio = (std::max)(1.0f, (std::min)(20.0f, ratio));

    const int N = lookaheadSamples_;
    float currentInputPeak = 0.0f;   // NEW: track input cycle peak
    float currentDelayedPeak = 0.0f; // NEW: track delayed output peak

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

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

                if (quantStep_ > 0.0f)
                {
                    if (useCeil_)
                        cvValue = std::ceil(cvValue / quantStep_) * quantStep_;
                    if (useFloor_)
                        cvValue = std::floor(cvValue / quantStep_) * quantStep_;
                    cvValue = (std::max)(0.0f, (std::min)(1.0f, cvValue));
                }

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

#ifdef FULL_WAVE_PEAK
        float valueForPeak = std::fabs(x);
#else
        float valueForPeak = x;
#endif
        if (valueForPeak > cyclePeak_)
            cyclePeak_ = valueForPeak;

        if (valueForPeak > currentInputPeak)   // track current input peak
            currentInputPeak = valueForPeak;

        lastSample_ = x;
        lookaheadBuffer_[bufferWritePos_] = x;

        int readPos = (bufferWritePos_ + 1) % N;
        float delayedSample = lookaheadBuffer_[readPos];
        out[s] = delayedSample;
        cvOut[s] = cvBuffer_[readPos];

        float delayedAbs = std::fabs(delayedSample);
        if (delayedAbs > currentDelayedPeak)
            currentDelayedPeak = delayedAbs;

        maxInOut[s] = currentInputPeak;       // output live peaks
        maxDelayedOut[s] = currentDelayedPeak;

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
