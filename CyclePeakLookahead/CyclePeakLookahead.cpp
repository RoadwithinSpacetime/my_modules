#define FULL_WAVE_PEAK
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
    , inputCyclePeakHold_(0.0f)
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
    inputCyclePeakHold_ = 0.0f;
    samplesSinceCycleStart_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // ~10 ms default
    minCycleGuard_ = lastPositiveWidth_ / 4;

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);   // ~30 ms
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    peakHoldBuffer_.assign(lookaheadSamples_, 0.0f);
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
    if (out)      std::memset(out, 0, sampleFrames * sizeof(float));
    if (cvOut)    std::memset(cvOut, 0, sampleFrames * sizeof(float));
    if (maxInOut) std::memset(maxInOut, 0, sampleFrames * sizeof(float));
    if (maxDelOut)std::memset(maxDelOut, 0, sampleFrames * sizeof(float));
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
    float ratio = pinRatio_;
    ratio = std::max(1.0f, std::min(20.0f, ratio));

    const int N = lookaheadSamples_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // Zero-cross detection (start of positive half-cycle)
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                // --- store peaks ---
                previousCyclePeak_ = cyclePeak_;
                inputCyclePeakHold_ = cyclePeak_; // hold for MaxInputCycle pin
                cyclePeak_ = 0.0f;

                // compute compressor CV
                float cvValue = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = previousCyclePeak_ - threshold;
                    if (over < 0.0f) over = 0.0f;
                    float compressedPeak = threshold + over / ratio;
                    cvValue = compressedPeak / previousCyclePeak_;
                    cvValue = std::clamp(cvValue, 0.0f, 1.0f);
                }

                // fill CV buffer and peak hold buffer for lookahead alignment
                int fillCount = cycleLength;
                if (fillCount > N) fillCount = N;
                for (int i = 1; i <= fillCount; ++i)
                {
                    int idx = bufferWritePos_ - i;
                    idx %= N;
                    if (idx < 0) idx += N;
                    cvBuffer_[idx] = cvValue;
                    peakHoldBuffer_[idx] = inputCyclePeakHold_;
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

        // lookahead delay readout
        lookaheadBuffer_[bufferWritePos_] = x;

        int readPos = (bufferWritePos_ + 1) % N;
        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = cvBuffer_[readPos];
        maxInOut[s] = inputCyclePeakHold_;
        maxDelOut[s] = peakHoldBuffer_[readPos];

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}

