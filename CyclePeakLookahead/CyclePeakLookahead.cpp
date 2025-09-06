#include "CyclePeakLookahead.h"
#include <cmath>

#undef max
#undef min

CyclePeakLookahead::CyclePeakLookahead()
    : bufferWritePos_(0)
    , lookaheadSamples_(0)
    , lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , samplesSinceCycleStart_(0)
    , sampleRate_(0.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;
    samplesSinceCycleStart_ = 0;

    // 30 ms lookahead
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1)
        lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    peakHoldBuffer_.assign(lookaheadSamples_, 0.0f);
    bufferWritePos_ = 0;

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinPeak_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    pinOut_.setStreaming(true);
    pinPeak_.setStreaming(true);
    setSubProcess(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* peakOut = getBuffer(pinPeak_);

    if (!in || !out || !peakOut)
        return;

    const int L = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        const float x = in[s];

        // 1) Output current delayed audio + scheduled held-peak
        out[s] = lookaheadBuffer_[bufferWritePos_];
        peakOut[s] = peakHoldBuffer_[bufferWritePos_];

        // 2) Detect positive zero-crossing (start of a new cycle)
        const bool posZero = (lastSample_ <= 0.0f && x > 0.0f);

        if (posZero)
        {
            // Previous cycle finished, schedule its peak into the lookahead peak buffer
            const int D = samplesSinceCycleStart_;

            if (D > 0)
            {
                int offset = L - D;
                int startK = offset < 0 ? -offset : 0;
                int nToWrite = D - startK;

                if (nToWrite > 0)
                {
                    int idx = (bufferWritePos_ + startK) % L;
                    for (int k = 0; k < nToWrite; ++k)
                    {
                        peakHoldBuffer_[idx] = cyclePeak_;
                        idx = (idx + 1) % L;
                    }
                }
            }

            // Reset for the new cycle
            samplesSinceCycleStart_ = 0;
            cyclePeak_ = 0.0f;
        }

        // 3) Update current cycle peak (full-wave)
        const float v = std::fabs(x);  // full-wave magnitude
        if (v > cyclePeak_)
            cyclePeak_ = v;

        // 4) Write current input sample into the audio delay
        lookaheadBuffer_[bufferWritePos_] = x;

        // 5) Advance ring and per-cycle counters
        bufferWritePos_ = (bufferWritePos_ + 1) % L;
        samplesSinceCycleStart_++;
        lastSample_ = x;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
