#include "CyclePeakLookahead.h"
#include <cstring>
#include <algorithm>

#undef max
#undef min

#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
    initializePin(pinRampLength_);
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

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // Update user parameters
    rampLength_ = std::max(1, pinRampLength_.getValue());

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
    if (out) memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut) memset(cvOut, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    if (!in || !out || !cvOut) return;

    float threshold = pinThreshold_ * 0.1f; // scale to volts
    float ratio = std::clamp(pinRatio_.getValue(), 1.0f, 20.0f);

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Trigger ramp BEFORE zero crossing ---
        int rampStart = lastPositiveWidth_ - rampLength_;
        if (!rampActive_
            && lastPositiveWidth_ > rampLength_
            && samplesSinceCycleStart_ >= rampStart)
        {
            // Compute next CV value ahead of crossing
            nextCvValue_ = 1.0f;
            if (cyclePeak_ > 0.0f)
            {
                float over = std::max(0.0f, cyclePeak_ - threshold);
                float compressed = threshold + over / ratio;
                nextCvValue_ = std::clamp(compressed / cyclePeak_, 0.0f, 1.0f);
            }

            rampSamplesRemaining_ = rampLength_;
            rampActive_ = true;
        }

        // --- Zero-cross detection ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;
                samplesSinceCycleStart_ = 0;

                // At crossing, ramp should be complete:
                if (!rampActive_)
                {
                    prevCvValue_ = nextCvValue_;
                }
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

        // --- CV ramp generation ---
        float cvValue;
        if (rampActive_)
        {
            // progress from old to new across rampLength_ samples
            int samplesIntoRamp = rampLength_ - rampSamplesRemaining_;
            float t = static_cast<float>(samplesIntoRamp) / (float)rampLength_;
            cvValue = prevCvValue_ + t * (nextCvValue_ - prevCvValue_);
            --rampSamplesRemaining_;

            if (rampSamplesRemaining_ <= 0)
            {
                rampActive_ = false;
                prevCvValue_ = nextCvValue_;
            }
        }
        else
        {
            cvValue = prevCvValue_;
        }

        // Write into lookahead buffer
        lookaheadBuffer_[bufferWritePos_] = x;
        cvBuffer_[bufferWritePos_] = cvValue;

        // Delayed output
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
