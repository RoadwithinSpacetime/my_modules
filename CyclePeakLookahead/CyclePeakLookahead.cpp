#include "CyclePeakLookahead.h"
#include <algorithm>
#include <cstring>
#undef max
#undef min

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
    , minSamplesBetweenUpdates_(0)
    , samplesSinceLastUpdate_(0)
    , smoothedCV_(1.0f)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
    initializePin(pinMaxFreq_);
    initializePin(pinAttack_);
    initializePin(pinRelease_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;
    previousCyclePeak_ = 0.0f;
    samplesSinceCycleStart_ = 0;
    samplesSinceLastUpdate_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_);
    minCycleGuard_ = lastPositiveWidth_ / 4;

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    bufferWritePos_ = 0;

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    if (!in || !out || !cvOut) return;

    // Parameters
    const float threshold = std::clamp(pinThreshold_.getValue(), 0.0f, 1.0f) * 0.1f;
    float ratio = std::clamp(pinRatio_.getValue(), 1.0f, 20.0f);
    float maxFreq = std::max(1.0f, pinMaxFreq_.getValue());  // avoid div0
    minSamplesBetweenUpdates_ = static_cast<int>(sampleRate_ / maxFreq);

    // Attack/Release smoothing coefficients
    const float attackMs = std::max(0.02f, pinAttack_.getValue());   
    const float releaseMs = std::max(0.02f, pinRelease_.getValue());
    float attackCoeff = 1.0f - std::exp(-1.0f / (sampleRate_ * (attackMs * 0.001f)));
    float releaseCoeff = 1.0f - std::exp(-1.0f / (sampleRate_ * (releaseMs * 0.001f)));

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;
        samplesSinceLastUpdate_++;

        // Detect positive zero-crossing
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                if (samplesSinceLastUpdate_ >= minSamplesBetweenUpdates_)
                {
                    samplesSinceLastUpdate_ = 0;

                    // Compute compressed CV
                    float cvValue = 1.0f;
                    if (previousCyclePeak_ > 0.0f)
                    {
                        float over = std::max(0.0f, previousCyclePeak_ - threshold);
                        float compressed = threshold + over / ratio;
                        cvValue = std::clamp(compressed / previousCyclePeak_, 0.0f, 1.0f);
                    }

                    // Retro-fill CV buffer for the just-finished cycle
                    int fillCount = std::min(samplesSinceCycleStart_, N);
                    for (int i = 1; i <= fillCount; ++i)
                    {
                        int idx = bufferWritePos_ - i;
                        idx %= N;
                        if (idx < 0) idx += N;
                        cvBuffer_[idx] = cvValue;
                    }
                }

                lastPositiveWidth_ = samplesSinceCycleStart_;
                minCycleGuard_ = lastPositiveWidth_ / 4;
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

        // Write audio into lookahead buffer
        lookaheadBuffer_[bufferWritePos_] = x;

        // Read delayed outputs
        int readPos = (bufferWritePos_ + 1) % N;
        float rawCV = cvBuffer_[readPos];

        // Attack/Release smoothing
        if (rawCV < smoothedCV_)
            smoothedCV_ += attackCoeff * (rawCV - smoothedCV_);
        else
            smoothedCV_ += releaseCoeff * (rawCV - smoothedCV_);

        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = smoothedCV_;

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
