#include "CyclePeakLookahead.h"

#undef max
#undef min // protect against Windows macros

#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
    : lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , previousCyclePeak_(0.0f)
    , bufferWritePos_(0)
    , lookaheadSamples_(0)
    , samplesSinceCycleStart_(0)
    , lastPositiveWidth_(0)
    , minCycleGuard_(0)
    , sampleRate_(0.0)
    , cvStart_(10.0f)
    , cvTarget_(10.0f)
    , cycleLength_(1)
    , cyclePos_(0)
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
    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // default 10ms
    minCycleGuard_ = lastPositiveWidth_ / 4;

    // 30 ms lookahead buffer
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
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

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);

    if (!in || !out || !cvOut)
        return;

    // Read controls
    float threshold = pinThreshold_ * 10.0f; // map 0–1 -> 0–10 V
    float ratio = pinRatio_;
    if (ratio < 1.0f) ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Detect new cycle on positive zero-crossing (with guard) ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                // Commit peak of last cycle
                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // Setup interpolation for the new cycle
                cvStart_ = cvTarget_;

                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;

                    float compressedPeak = threshold + over / ratio;

                    // Normalize to input peak, scale to 0–10 V
                    cvTarget_ = 10.0f * (compressedPeak / previousCyclePeak_);
                }
                else
                {
                    cvTarget_ = 10.0f; // idle = max CV
                }

                if (cvTarget_ < 0.0f) cvTarget_ = 0.0f;
                if (cvTarget_ > 10.0f) cvTarget_ = 10.0f;

                cycleLength_ = samplesSinceCycleStart_;
                cyclePos_ = 0;

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

        // --- Write to lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- Read delayed sample ---
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];

        // --- Interpolated CV output ---
        float t = (cycleLength_ > 0) ? (float)cyclePos_ / (float)cycleLength_ : 1.0f;
        float cvValue = cvStart_ + (cvTarget_ - cvStart_) * t;

        if (cvValue < 0.0f) cvValue = 0.0f;
        if (cvValue > 10.0f) cvValue = 10.0f;

        cvOut[s] = cvValue;
        cyclePos_++;

        // --- Increment buffer write position ---
        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
