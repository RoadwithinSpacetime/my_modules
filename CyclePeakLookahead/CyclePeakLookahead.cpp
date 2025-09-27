#include "CyclePeakLookahead.h"
#include <cstring>
#undef max
#undef min

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    , maxInputCycle_(0.0f)
    , sampleRate_(0.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinMaxInputCycle_);
    initializePin(pinMaxDelayedCycle_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;
    previousCyclePeak_ = 0.0f;
    maxInputCycle_ = 0.0f;
    samplesSinceCycleStart_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_);
    minCycleGuard_ = lastPositiveWidth_ / 4;

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    delayedPeakBuffer_.assign(lookaheadSamples_, 0.0f);
    bufferWritePos_ = 0;

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    pinMaxInputCycle_.setStreaming(true);
    pinMaxDelayedCycle_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
    pinMaxInputCycle_.setStreaming(true);
    pinMaxDelayedCycle_.setStreaming(true);
    setSubProcess(&CyclePeakLookahead::subProcess);
}

void CyclePeakLookahead::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    float* maxInOut = getBuffer(pinMaxInputCycle_);
    float* maxDelOut = getBuffer(pinMaxDelayedCycle_);
    if (out)       std::memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut)     std::memset(cvOut, 0, sampleFrames * sizeof(float));
    if (maxInOut)  std::memset(maxInOut, 0, sampleFrames * sizeof(float));
    if (maxDelOut) std::memset(maxDelOut, 0, sampleFrames * sizeof(float));
}

void CyclePeakLookahead::subProcess(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    float* cvOut = getBuffer(pinCV_);
    float* maxInOut = getBuffer(pinMaxInputCycle_);
    float* maxDelOut = getBuffer(pinMaxDelayedCycle_);
    if (!in || !out || !cvOut || !maxInOut || !maxDelOut) return;

    float threshold = pinThreshold_ * 0.1f; // 0–1 => 0–10V
    float ratio = std::clamp((float)pinRatio_, 1.0f, 20.0f);
    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Zero-cross detection (positive going) ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                maxInputCycle_ = cyclePeak_; // latch true input cycle peak
                cyclePeak_ = 0.0f;

                // Compressor CV
                float cvValue = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = std::max(0.0f, previousCyclePeak_ - threshold);
                    float compressedPeak = threshold + over / ratio;
                    cvValue = std::clamp(compressedPeak / previousCyclePeak_, 0.0f, 1.0f);
                }

                // Backfill lookahead buffers
                int fillCount = std::min(cycleLength, N);
                for (int i = 1; i <= fillCount; ++i)
                {
                    int idx = bufferWritePos_ - i;
                    idx %= N;
                    if (idx < 0) idx += N;

                    cvBuffer_[idx] = cvValue;
                    delayedPeakBuffer_[idx] = maxInputCycle_; // delayed copy
                }

                samplesSinceCycleStart_ = 0;
            }
        }

#ifdef FULL_WAVE_PEAK
        float v = std::fabs(x);
#else
        float v = x;
#endif
        if (v > cyclePeak_)
            cyclePeak_ = v;

        lastSample_ = x;

        // --- Lookahead read ---
        lookaheadBuffer_[bufferWritePos_] = x;
        int readPos = (bufferWritePos_ + 1) % N;

        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = cvBuffer_[readPos];
        maxInOut[s] = maxInputCycle_;           // immediate peak
        maxDelOut[s] = delayedPeakBuffer_[readPos]; // delayed peak

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
