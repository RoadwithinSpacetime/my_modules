#include "CyclePeakLookahead.h"
#include <algorithm>
#include <cstring>
#include <cmath>
#include <random>

#undef max
#undef min

#define FULL_WAVE_PEAK

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

CyclePeakLookahead::CyclePeakLookahead()
    : bufferWritePos_(0)
    , lookaheadSamples_(0)
    , lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , previousCyclePeak_(0.0f)
    , samplesSinceCycleStart_(0)
    , lastPositiveWidth_(0)
    , minCycleGuard_(0)
    , rampLength_(5)
    , prevCvValue_(1.0f)
    , nextCvValue_(1.0f)
    , rampSamplesRemaining_(0)
    , sampleRate_(0.0)
    , releaseMin_(1)
    , ditherAmount_(-90.0f)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
    initializePin(pinRamp_);
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

    // Random generator for dither
    std::random_device rd;
    rng_ = std::mt19937(rd());
    ditherDist_ = std::uniform_real_distribution<float>(-1.0f, 1.0f);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    setSubProcess(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    if (!in || !out || !cvOut) return;

    float threshold = pinThreshold_ * 0.1f;
    float ratio = std::clamp(pinRatio_.getValue(), 1.0f, 20.0f);
    rampLength_ = static_cast<int>(std::clamp(pinRamp_.getValue(), 1.0f, 100.0f));
    releaseMin_ = static_cast<int>(std::clamp(pinRelease_.getValue(), 1.0f, 1000.0f));

    const int N = lookaheadSamples_;

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

                nextCvValue_ = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = std::max(0.0f, previousCyclePeak_ - threshold);
                    float compressed = threshold + over / ratio;
                    nextCvValue_ = std::clamp(compressed / previousCyclePeak_, 0.0f, 1.0f);
                }

                rampSamplesRemaining_ = rampLength_;
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

        float cvValue;
        if (rampSamplesRemaining_ > 0)
        {
            int samplesIntoRamp = rampLength_ - rampSamplesRemaining_;
            float t = static_cast<float>(samplesIntoRamp) / rampLength_;
            // Hann ramp
            float hannT = 0.5f * (1.0f - std::cos(M_PI * t));
            cvValue = prevCvValue_ + hannT * (nextCvValue_ - prevCvValue_);
            --rampSamplesRemaining_;

            if (rampSamplesRemaining_ == 0)
                prevCvValue_ = nextCvValue_;
        }
        else
        {
            cvValue = prevCvValue_;
        }

        // Add very small dither
        float dither = std::pow(10.0f, ditherAmount_ / 20.0f) * ditherDist_(rng_);
        cvValue += dither;

        cvBuffer_[bufferWritePos_] = cvValue;

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

