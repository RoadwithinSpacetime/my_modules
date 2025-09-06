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

        // --- write current input into lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- full-wave peak detection ---
        const float v = std::fabs(x);
        if (v > cyclePeak_)
            cyclePeak_ = v;

        // --- write peak into lookahead peak buffer (sample-and-hold) ---
        peakHoldBuffer_[bufferWritePos_] = cyclePeak_;

        // --- detect positive zero-crossing to start a new cycle ---
        const bool posZero = (lastSample_ <= 0.0f && x > 0.0f);
        if (posZero)
        {
            cyclePeak_ = 0.0f; // reset for new cycle
        }

        // --- output delayed audio and peak ---
        out[s] = lookaheadBuffer_[bufferWritePos_];
        peakOut[s] = peakHoldBuffer_[bufferWritePos_];

        // --- advance ring buffer ---
        bufferWritePos_ = (bufferWritePos_ + 1) % L;
        lastSample_ = x;
    }
}


// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
