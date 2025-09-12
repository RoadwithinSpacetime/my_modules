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
    , cvTarget_(1.0f)
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

    // 30 ms lookahead buffer
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    bufferWritePos_ = 0;

    // start silent until input streams
    setSubProcess(&CyclePeakLookahead::subProcessSilent);
    pinOut_.setStreaming(false);
    pinCV_.setStreaming(false);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    if (pinIn_.isStreaming())
    {
        setSubProcess(&CyclePeakLookahead::subProcess);
        pinOut_.setStreaming(true);
        pinCV_.setStreaming(true);
    }
    else
    {
        setSubProcess(&CyclePeakLookahead::subProcessSilent);
        pinOut_.setStreaming(false);
        pinCV_.setStreaming(false);
    }
}

void CyclePeakLookahead::subProcessSilent(int sampleFrames)
{
    float* in = getBuffer(pinIn_);

    // If audio arrives again -> wake up
    if (in && pinIn_.isStreaming())
    {
        setSubProcess(&CyclePeakLookahead::subProcess);
        pinOut_.setStreaming(true);
        pinCV_.setStreaming(true);

        subProcess(sampleFrames);
        return;
    }

    // Output silence while idle
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
    if (!in || !out || !cvOut)
        return;

    // Read controls
    float threshold = pinThreshold_ * 0.1f; // map 0–1 -> 0–10 V
    float ratio = pinRatio_;
    if (ratio < 1.0f)  ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    // Frequency guard thresholds (30 Hz – 6 kHz)
    const int minSamples = static_cast<int>(sampleRate_ / 6000.0); // shortest half-cycle allowed
    const int maxSamples = static_cast<int>(sampleRate_ / 30.0);   // longest half-cycle allowed

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Detect new cycle on positive zero-crossing ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            // Minimum guard to avoid false triggers
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                samplesSinceCycleStart_ = 0;

                // Enforce 30 Hz–6 kHz range
                if (cycleLength >= minSamples && cycleLength <= maxSamples)
                {
                    lastPositiveWidth_ = cycleLength;
                    minCycleGuard_ = lastPositiveWidth_ / 4;

                    // Commit peak of last valid cycle
                    previousCyclePeak_ = cyclePeak_;
                    cyclePeak_ = 0.0f;

                    if (previousCyclePeak_ > 0.0f)
                    {
                        float over = previousCyclePeak_ - threshold;
                        if (over < 0.0f) over = 0.0f;

                        float compressedPeak = threshold + over / ratio;
                        cvTarget_ = compressedPeak / previousCyclePeak_;
                    }
                    else
                    {
                        cvTarget_ = 1.0f;
                    }

                    if (cvTarget_ < 0.0f) cvTarget_ = 0.0f;
                    if (cvTarget_ > 1.0f) cvTarget_ = 1.0f;
                }
                else
                {
                    // Out-of-band cycle -> hold previous CV and reset peak
                    cyclePeak_ = 0.0f;
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

        // --- Lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];

        // --- CV output (sample-and-hold) ---
        cvOut[s] = cvTarget_;

        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
