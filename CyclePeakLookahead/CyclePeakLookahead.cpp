#include "CyclePeakLookahead.h"
#include <cstring>   // memset
#include <algorithm> // std::clamp
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
    , cvOversamplePos_(0)
    , cvFilterState_(1.0f)
    , cvFilterCoeff_(0.0f)
    , sampleRate_(0.0)
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

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_);
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    bufferWritePos_ = 0;

    // Oversample CV buffer: 4× lookahead length
    cvOversampleBuffer_.assign(lookaheadSamples_ * oversampleFactor_, 1.0f);
    cvOversamplePos_ = 0;

    // 1-pole filter at ~5 kHz for CV decimation
    // adjust cutoff as needed (here 5 kHz is safe for 48 kHz audio)
    float cutoff = 5000.0f;
    float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff);
    cvFilterCoeff_ = 1.0f - std::exp(-1.0f / (rc * sampleRate_ * oversampleFactor_));

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

void CyclePeakLookahead::subProcessSilent(int sampleFrames)
{
    float* in = getBuffer(pinIn_);
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
    if (!in || !out || !cvOut) return;

    float threshold = pinThreshold_ * 0.1f; // map 0–1 -> 0–10V
    float ratio = std::clamp(static_cast<float>(pinRatio_), 1.0f, 20.0f);


    const int N = lookaheadSamples_;
    const int N4 = N * oversampleFactor_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];

        // -----------------------
        // 4× CV oversampling loop
        // -----------------------
        for (int k = 0; k < oversampleFactor_; ++k)
        {
            samplesSinceCycleStart_++;

            // Zero-cross detection
            if (lastSample_ <= 0.0f && x > 0.0f)
            {
                if (samplesSinceCycleStart_ > minCycleGuard_)
                {
                    int cycleLength = samplesSinceCycleStart_;
                    lastPositiveWidth_ = cycleLength;
                    minCycleGuard_ = lastPositiveWidth_ / 4;

                    previousCyclePeak_ = cyclePeak_;
                    cyclePeak_ = 0.0f;

                    float cvValue = 1.0f;
                    if (previousCyclePeak_ > 0.0f)
                    {
                        float over = previousCyclePeak_ - threshold;
                        if (over < 0.0f) over = 0.0f;
                        float compressedPeak = threshold + over / ratio;
                        cvValue = compressedPeak / previousCyclePeak_;
                        cvValue = std::clamp(cvValue, 0.0f, 1.0f);
                    }

                    // retro-fill oversampled CV buffer
                    int fillCount = cycleLength * oversampleFactor_;
                    if (fillCount > N4) fillCount = N4;
                    for (int i = 1; i <= fillCount; ++i)
                    {
                        int idx = cvOversamplePos_ - i;
                        idx %= N4;
                        if (idx < 0) idx += N4;
                        cvOversampleBuffer_[idx] = cvValue;
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

            // advance oversample write position
            cvOversamplePos_ = (cvOversamplePos_ + 1) % N4;
        }

        lastSample_ = x;

        // ----------------------------
        // Downsample CV (one-pole LPF)
        // ----------------------------
        int oversampleReadPos = (cvOversamplePos_ + 1) % N4;
        float rawCV = cvOversampleBuffer_[oversampleReadPos];
        cvFilterState_ += cvFilterCoeff_ * (rawCV - cvFilterState_);
        float filteredCV = cvFilterState_;

        // write audio lookahead
        lookaheadBuffer_[bufferWritePos_] = x;

        int readPos = (bufferWritePos_ + 1) % N;
        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = filteredCV;

        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
