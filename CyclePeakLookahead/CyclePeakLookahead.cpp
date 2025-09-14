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
    , cvCurrent_(1.0f)
    , cvPending_(1.0f)
    , compressionArmed_(false)
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

    // Wake up if audio resumes
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

    float threshold = pinThreshold_ * 0.1f; // map 0–1 -> 0–10 V
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
                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                // Store the cycle peak
                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // Calculate potential new CV
                float newCV = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;
                    newCV = compressedPeak / previousCyclePeak_;
                    if (newCV < 0.0f) newCV = 0.0f;
                    if (newCV > 1.0f) newCV = 1.0f;
                }

                // Arm compression only after first cycle above threshold
                if (!compressionArmed_)
                {
                    if (previousCyclePeak_ > threshold)
                    {
                        compressionArmed_ = true;
                        cvPending_ = newCV; // Prepare but don't apply yet
                    }
                    else
                    {
                        // Hold unity until armed
                        cvPending_ = 1.0f;
                    }
                }
                else
                {
                    // Normal operation: commit previous measurement
                    cvCurrent_ = cvPending_;
                    cvPending_ = newCV;
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

        lastSample_ = x;

        // --- Lookahead buffer for delayed audio ---
        lookaheadBuffer_[bufferWritePos_] = x;
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];

        // --- CV stair-step output ---
        cvOut[s] = cvCurrent_;

        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
