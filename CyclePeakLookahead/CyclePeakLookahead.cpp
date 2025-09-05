#include "CyclePeakLookahead.h"

using namespace gmpi;

CyclePeakLookahead::CyclePeakLookahead()
    : lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , bufferWritePos_(0)
    , lookaheadSamples_(0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinPeak_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate(); // <-- set sample rate

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;

    // Compute lookahead buffer size (30 ms)
    lookaheadSamples_ = static_cast<int>(0.03f * sampleRate_);
    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
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

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // --- Cycle peak detection ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            cyclePeak_ = 0.0f; // start new cycle
        }

        if (x > cyclePeak_)
            cyclePeak_ = x;

        lastSample_ = x;

        // --- Write to lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;
        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;

        // --- Output ---
        out[s] = lookaheadBuffer_[bufferWritePos_];    // delayed by 30 ms
        peakOut[s] = cyclePeak_;                       // current cycle peak
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
