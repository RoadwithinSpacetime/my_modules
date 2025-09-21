#include "CyclePeakLookahead.h"
#include <algorithm>
#include <cstring>
#include <cmath>
#undef max
#undef min

// Ensure a PI constant is available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
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
    if (out)  memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut) memset(cvOut, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    if (!in || !out || !cvOut) return;

    float threshold = pinThreshold_ * 0.1f; // 0–1 -> 0–10 V
    float ratio = std::clamp(pinRatio_.getValue(), 1.0f, 20.0f);

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        //--- Zero-cross detection (start of NEXT cycle) ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // compute next CV value
                nextCvValue_ = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = std::max(0.0f, previousCyclePeak_ - threshold);
                    float compressed = threshold + over / ratio;
                    nextCvValue_ = std::clamp(compressed / previousCyclePeak_, 0.0f, 1.0f);
                }

                // schedule ramp to END at this zero crossing
                // ramp starts rampLength_ samples BEFORE crossing
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

        // --- Ramp generation (Hann taper) ---
        float cvValue;
        if (rampSamplesRemaining_ > 0)
        {
            // progress from start->target over rampLength_ samples
            int samplesIntoRamp = rampLength_ - rampSamplesRemaining_;
            float t = static_cast<float>(samplesIntoRamp) / static_cast<float>(rampLength_);
            // Hann curve for smoother spectrum
            float w = 0.5f * (1.0f - std::cos(static_cast<float>(M_PI) * t));
            cvValue = prevCvValue_ + w * (nextCvValue_ - prevCvValue_);
            --rampSamplesRemaining_;
            if (rampSamplesRemaining_ == 0)
                prevCvValue_ = nextCvValue_;
        }
        else
        {
            cvValue = prevCvValue_;
        }

        // write into circular buffers
        lookaheadBuffer_[bufferWritePos_] = x;
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
