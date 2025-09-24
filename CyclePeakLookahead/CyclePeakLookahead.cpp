#include "CyclePeakLookahead.h"
#include <cstring>

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
    initializePin(pinRampLength_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;
    previousCyclePeak_ = 0.0f;
    samplesSinceCycleStart_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // initial guess ~10 ms
    minCycleGuard_ = lastPositiveWidth_ / 4;

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1)
        lookaheadSamples_ = 1;

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
    // still read ramp length for completeness, but unused
    int userRamp = pinRampLength_.getValue();
    rampLength_ = (std::max)(1, userRamp);

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
    if (out)   memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut) memset(cvOut, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    if (!in || !out || !cvOut) return;

    float threshold = pinThreshold_ * 0.1f; // 0–1 to 0–10 V
    float ratio = std::clamp(pinRatio_.getValue(), 1.0f, 20.0f);

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // ---- Predict zero crossing and compute next CV ----
        // (We still compute nextCvValue_ as soon as rampStart is reached)
        int predictedEnd = lastPositiveWidth_;
        int rampStart = predictedEnd - rampLength_;
        if (predictedEnd > rampLength_ && samplesSinceCycleStart_ >= rampStart)
        {
            float over = std::fmax(0.0f, cyclePeak_ - threshold);
            float compressed = threshold + over / ratio;
            nextCvValue_ = (cyclePeak_ > 0.0f)
                ? std::clamp(compressed / cyclePeak_, 0.0f, 1.0f)
                : 1.0f;
        }

        // ---- Zero-cross detection ----
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4;
                previousCyclePeak_ = cyclePeak_;
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

        // --- write audio into lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- Immediate CV output (NO ramp) ---
        float cvValue = nextCvValue_;
        prevCvValue_ = nextCvValue_;

        // --- optional ceiling quantisation ---
        if (useCeil_ && ceilStep_ > 0.0f)
        {
            cvValue = std::ceil(cvValue / ceilStep_) * ceilStep_;
            if (cvValue > 1.0f) cvValue = 1.0f;
        }

        cvBuffer_[bufferWritePos_] = cvValue;

        // --- delayed output for lookahead audio ---
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
