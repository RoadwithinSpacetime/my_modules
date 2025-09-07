#include "CyclePeakLookahead.h"

#undef max
#undef min
#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
    : lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , bufferWritePos_(0)
    , lookaheadSamples_(0)
    , samplesSinceCycleStart_(0)
    , lastPositiveWidth_(0)
    , minCycleGuard_(0)
    , sampleRate_(0.0) // default before open()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinGate_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;

    // 30 ms lookahead buffer
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    peakHoldBuffer_.assign(lookaheadSamples_, 0.0f);
    bufferWritePos_ = 0;

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinGate_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    pinOut_.setStreaming(true);
    pinGate_.setStreaming(true);
    setSubProcess(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* gateOut = getBuffer(pinGate_);

    if (!in || !out || !gateOut)
        return;

    // read inputs
    float threshold = pinThreshold_;
    float ratio = pinRatio_;

    // clamp ratio
    if (ratio < 1.0f) ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // --- detect new cycle on positive zero-crossing ---
        if (lastSample_ <= 0.0f && x > 0.0f && samplesSinceCycleStart_ > minCycleGuard_)
        {
            // schedule peak for this cycle
            int writeIndex = (bufferWritePos_ + lookaheadSamples_ - samplesSinceCycleStart_) % lookaheadSamples_;
            peakHoldBuffer_[writeIndex] = cyclePeak_;

            // prepare next cycle
            lastPositiveWidth_ = samplesSinceCycleStart_;
            minCycleGuard_ = std::max(1, lastPositiveWidth_ / 4);
            samplesSinceCycleStart_ = 0;
            cyclePeak_ = 0.0f;
        }

#ifdef FULL_WAVE_PEAK
        float valueForPeak = std::fabs(x);
#else
        float valueForPeak = x;
#endif
        if (valueForPeak > cyclePeak_)
            cyclePeak_ = valueForPeak;

        lastSample_ = x;
        ++samplesSinceCycleStart_;

        // --- write to lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- read delayed audio and peak ---
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];
        float currentPeak = peakHoldBuffer_[readPos];

        // --- compressor-like CV output ---
        float cvOut = 10.0f; // base at threshold
        if (currentPeak > threshold)
        {
            float over = currentPeak - threshold;
            float reduction = over * (1.0f - 1.0f / ratio); // compressor law
            cvOut = 10.0f - reduction;
            if (cvOut < 0.0f) cvOut = 0.0f;
        }
        gateOut[s] = cvOut;

        // --- increment buffer write position ---
        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
