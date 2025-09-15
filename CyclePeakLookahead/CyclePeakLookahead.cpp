#include "CyclePeakLookahead.h"
#include <algorithm> // clamp
#include <cstring>   // memset
#undef max
#undef min  // protect against Windows macros

// Uncomment to enable full-wave peak detection
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

    // 30 ms lookahead (audio + CV buffers must be same length)
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f); // unity CV by default
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

    // threshold mapping: you previously used *0.1 (0..1 -> 0..10V), keep that mapping
    float threshold = pinThreshold_ * 0.1f;
    float ratio = pinRatio_;
    if (ratio < 1.0f) ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Detect positive zero-crossing (start of a new input cycle) ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            // only treat as new cycle if the previous half-cycle was long enough
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                // cycleLength is the number of samples of the cycle that just finished
                int cycleLength = samplesSinceCycleStart_;

                // update guards
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                // previousCyclePeak_ holds the peak of the cycle that just finished
                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // --- compute compressed CV for that just-finished cycle ---
                float cvValue = 1.0f; // unity by default
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;

                    // normalize compressedPeak relative to measured peak => 0..1
                    cvValue = compressedPeak / previousCyclePeak_;
                    if (cvValue < 0.0f) cvValue = 0.0f;
                    if (cvValue > 1.0f) cvValue = 1.0f;
                }

                // --- RETRO-FILL the CV buffer for the samples belonging to the cycle that just finished ---
                // The samples of that cycle are at indices (bufferWritePos_ - 1), (bufferWritePos_ - 2), ..., (bufferWritePos_ - cycleLength)
                // Clamp cycleLength so we don't overwrite more than the buffer.
                int fillCount = cycleLength;
                if (fillCount > N) fillCount = N;

                for (int i = 1; i <= fillCount; ++i)
                {
                    int idx = bufferWritePos_ - i;
                    idx %= N;
                    if (idx < 0) idx += N;
                    cvBuffer_[idx] = cvValue;
                }

                // reset sample counter for next cycle
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

        // --- Write current audio sample into lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // Ensure current write slot has a sensible CV (hold last known CV if not yet retro-filled)
        // We use the last cv stored at this slot (it may have been set by a previous retro-fill), so no overwrite here.
        // If you prefer to force current slot to unity until retro-fill, uncomment next line:
        // cvBuffer_[bufferWritePos_] = cvBuffer_[bufferWritePos_]; // no-op (kept for clarity)

        // --- Read delayed outputs (audio and CV) from same read position ---
        int readPos = (bufferWritePos_ + 1) % N;
        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = cvBuffer_[readPos];

        // --- advance write pointer ---
        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
