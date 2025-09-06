#include "CyclePeakLookahead.h"

#undef max
#undef min  // protect against Windows macros

// Uncomment to enable full-wave peak detection
#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
    : lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , previousCyclePeak_(0.0f)
    , bufferWritePos_(0)
    , lookaheadSamples_(0)
    , samplesSinceCycleStart_(0)
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
    previousCyclePeak_ = 0.0f;
    samplesSinceCycleStart_ = 0;

    // 30 ms lookahead buffer
    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
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

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // --- Detect new cycle on positive zero-crossing ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            // Finalize previous cycle peak
            previousCyclePeak_ = cyclePeak_;
            cyclePeak_ = 0.0f;  // reset for new cycle
            samplesSinceCycleStart_ = 0; // reset cycle sample counter
        }

#ifdef FULL_WAVE_PEAK
        float valueForPeak = std::fabs(x);
#else
        float valueForPeak = x;
#endif

        // Update cycle peak
        if (valueForPeak > cyclePeak_)
            cyclePeak_ = valueForPeak;

        lastSample_ = x;
        samplesSinceCycleStart_++;

        // --- Write to lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;

        // --- Write previous cycle peak to peak buffer (aligned to delayed audio) ---
        peakHoldBuffer_[bufferWritePos_] = previousCyclePeak_;

        // --- Read delayed sample and peak ---
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];
        peakOut[s] = peakHoldBuffer_[readPos];

        // --- Increment buffer write position ---
        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
