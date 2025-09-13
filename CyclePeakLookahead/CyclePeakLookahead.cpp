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
    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // default 10ms
    minCycleGuard_ = lastPositiveWidth_ / 4;

    // 30 ms lookahead buffer
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;
    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);

    // CV buffer must be same length so reads align
    cvBuffer_.assign(lookaheadSamples_, 1.0f); // default unity (no compression)

    bufferWritePos_ = 0;

    // Start silent until input is streaming
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

    if (!in || !out || !cvOut)
        return;

    // Controls: note the threshold scale you prefer (0.1 or 10.0) — using *0.1 per your last working setup
    float threshold = pinThreshold_ * 0.1f; // map 0–1 -> 0–10 V (your mapping)
    float ratio = pinRatio_;
    if (ratio < 1.0f) ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Detect positive zero-crossing with guard ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                // capture cycle length BEFORE resetting
                int cycleLength = samplesSinceCycleStart_;

                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                // Commit peak of last cycle
                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // --- Compute new stair-step CV (normalized 0..1) ---
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;

                    float newCv = compressedPeak / previousCyclePeak_;
                    if (newCv < 0.0f) newCv = 0.0f;
                    if (newCv > 1.0f) newCv = 1.0f;

                    cvTarget_ = newCv;

                    // --- IMPORTANT: retroactively fill the CV buffer slots that contain the just-completed cycle ---
                    // The last 'cycleLength' samples were already written into lookaheadBuffer at indices:
                    // (bufferWritePos_-1), (bufferWritePos_-2), ..., so write cvTarget_ into the same indices
                    // so when these audio samples are later read out, the CV matches them.
                    if (lookaheadSamples_ > 0)
                    {
                        int maxFill = cycleLength;
                        if (maxFill > lookaheadSamples_) maxFill = lookaheadSamples_;
                        for (int i = 1; i <= maxFill; ++i)
                        {
                            int idx = bufferWritePos_ - i;
                            // wrap
                            idx %= lookaheadSamples_;
                            if (idx < 0) idx += lookaheadSamples_;
                            cvBuffer_[idx] = cvTarget_;
                        }
                    }
                }
                else
                {
                    // No signal in previous cycle -> unity CV
                    cvTarget_ = 1.0f;

                    // Fill past cycle slots with unity
                    if (lookaheadSamples_ > 0)
                    {
                        int maxFill = samplesSinceCycleStart_;
                        if (maxFill > lookaheadSamples_) maxFill = lookaheadSamples_;
                        for (int i = 1; i <= maxFill; ++i)
                        {
                            int idx = bufferWritePos_ - i;
                            idx %= lookaheadSamples_;
                            if (idx < 0) idx += lookaheadSamples_;
                            cvBuffer_[idx] = cvTarget_;
                        }
                    }
                }

                // reset counter for next cycle
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

        // --- Write audio into lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // Also write current cvTarget_ into current write slot so future samples will be consistent
        cvBuffer_[bufferWritePos_] = cvTarget_;

        // --- Read delayed sample (same scheme as earlier) ---
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];

        // --- Output the CV that corresponds to that delayed audio sample (synchronized) ---
        cvOut[s] = cvBuffer_[readPos];

        // increment write position
        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
