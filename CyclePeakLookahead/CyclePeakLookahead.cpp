#include "CyclePeakLookahead.h"
#include <algorithm> // std::max
#include <cstring>   // memset

#undef max
#undef min
#define FULL_WAVE_PEAK   // comment out for half-wave peak

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

    // 30 ms look-ahead buffer
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

    float threshold = pinThreshold_ * 0.1f;  // 0..1 -> 0..10 V
    float ratio = pinRatio_;
    if (ratio < 1.0f) ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    static float cvHold = 1.0f;  // value held for current cycle

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // --- Detect new cycle on positive zero-crossing ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            // Compute compression for the cycle that just ended
            previousCyclePeak_ = cyclePeak_;
            cyclePeak_ = 0.0f;

            if (previousCyclePeak_ > 0.0f)
            {
                float over = previousCyclePeak_ - threshold;
                if (over < 0.0f) over = 0.0f;
                float compressed = threshold + over / ratio;

                cvHold = compressed / previousCyclePeak_;
                if (cvHold < 0.0f) cvHold = 0.0f;
                if (cvHold > 1.0f) cvHold = 1.0f;
            }
            else
            {
                cvHold = 1.0f; // no reduction when silent
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

        // --- Write to look-ahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- Read delayed sample ---
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];

        // --- Output held CV ---
        cvOut[s] = cvHold;

        // --- Increment buffer write position ---
        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
