#include "CyclePeakLookahead.h"

#undef max
#undef min  // protect against Windows macros

#define FULL_WAVE_PEAK

CyclePeakLookahead::CyclePeakLookahead()
    : lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , bufferWritePos_(0)
    , lookaheadSamples_(0)
    , lastPositiveWidth_(0)
    , minCycleGuard_(0)
    , samplesSinceCycleStart_(0)
    , sampleRate_(44100.0) // default initial value
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

    // Read control pins and clamp
    float threshold = pinThreshold_;
    float ratio = pinRatio_;
    if (ratio < 1.0f) ratio = 1.0f;
    if (ratio > 20.0f) ratio = 20.0f;

    static float previousCyclePeak = 0.0f;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // --- Detect new cycle on positive zero-crossing ---
        samplesSinceCycleStart_++;

        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            // guard against short half-cycles
            if (samplesSinceCycleStart_ >= minCycleGuard_)
            {
                previousCyclePeak = cyclePeak_;
                cyclePeak_ = 0.0f;
                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4; // quarter length
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

        // --- Lookahead buffer ---
        lookaheadBuffer_[bufferWritePos_] = x;
        int readPos = (bufferWritePos_ + 1) % lookaheadSamples_;
        out[s] = lookaheadBuffer_[readPos];

        // --- CV output ---
        float currentPeak = previousCyclePeak; // delayed per-cycle peak

        float over = currentPeak - threshold;
        if (over < 0.0f) over = 0.0f;
        float compressed = over / ratio;
        float cvValue = 10.0f - compressed * 10.0f; // scale to 0-10V
        if (cvValue < 0.0f) cvValue = 0.0f;
        if (cvValue > 10.0f) cvValue = 10.0f;

        cvOut[s] = cvValue;

        bufferWritePos_ = (bufferWritePos_ + 1) % lookaheadSamples_;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
