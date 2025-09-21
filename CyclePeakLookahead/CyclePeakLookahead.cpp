#include "CyclePeakLookahead.h"
#include <algorithm>
#include <cstring>
#include <cmath>
#include <random>

#undef max
#undef min

// Ensure a PI constant is available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FULL_WAVE_PEAK

// Default dielectric pole time-constants (seconds) and weights
static const float default_taus[] = { 0.002f, 0.01f, 0.05f, 0.5f }; // fast -> slow
static const float default_weights[] = { 0.45f, 0.30f, 0.15f, 0.10f }; // sum = 1.0

CyclePeakLookahead::CyclePeakLookahead()
    : bufferWritePos_(0)
    , lookaheadSamples_(0)
    , lastSample_(0.0f)
    , cyclePeak_(0.0f)
    , previousCyclePeak_(0.0f)
    , samplesSinceCycleStart_(0)
    , lastPositiveWidth_(0)
    , minCycleGuard_(0)
    , rampLength_(5)
    , prevCvValue_(1.0f)
    , nextCvValue_(1.0f)
    , rampSamplesRemaining_(0)
    , sampleRate_(0.0)
    , rng_(std::random_device{}())
    , uniDist_(-1.0f, 1.0f)
    , cachedDitherScale_(0.0f)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinCV_);
    initializePin(pinThreshold_);
    initializePin(pinRatio_);
    initializePin(pinRampLength_);
    initializePin(pinDitherDb_);
    initializePin(pinEnableDielectric_);
    initializePin(pinHarmonicMix_);

    // dielectricPoles_ will be initialized in open() if enabled
}

int32_t CyclePeakLookahead::open()
{
    sampleRate_ = getSampleRate();

    lastSample_ = 0.0f;
    cyclePeak_ = 0.0f;
    previousCyclePeak_ = 0.0f;
    samplesSinceCycleStart_ = 0;

    lastPositiveWidth_ = static_cast<int>(0.01 * sampleRate_); // default 10 ms width
    minCycleGuard_ = lastPositiveWidth_ / 4;

    lookaheadSamples_ = static_cast<int>(0.03 * sampleRate_); // 30 ms lookahead
    if (lookaheadSamples_ < 1) lookaheadSamples_ = 1;

    lookaheadBuffer_.assign(lookaheadSamples_, 0.0f);
    cvBuffer_.assign(lookaheadSamples_, 1.0f);
    bufferWritePos_ = 0;

    // init dielectric poles (inactive until enabled)
    dielectricPoles_.clear();

    // compute initial dither scale from default pin value (-90 dB)
    float ditherDb = static_cast<float>(pinDitherDb_.getValue()); // if pin has default -90
    cachedDitherScale_ = std::pow(10.0f, ditherDb / 20.0f);

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);

    return MpBase2::open();
}

void CyclePeakLookahead::onSetPins()
{
    // update parameters when pins change
    rampLength_ = std::max(1, static_cast<int>(std::round(pinRampLength_.getValue())));
    // clamp ramp length to reasonable range
    rampLength_ = std::min(std::max(rampLength_, 1), 2048);

    float ditherDb = pinDitherDb_.getValue();
    // clamp dither dB -120 .. -20
    if (ditherDb < -120.0f) ditherDb = -120.0f;
    if (ditherDb > -20.0f) ditherDb = -20.0f;
    cachedDitherScale_ = std::pow(10.0f, ditherDb / 20.0f);

    // (re)initialise dielectric poles if user enabled
    if (pinEnableDielectric_.getValue() > 0.5f)
    {
        dielectricPoles_.clear();
        for (size_t i = 0; i < sizeof(default_taus) / sizeof(default_taus[0]); ++i)
        {
            Pole p;
            float tau = default_taus[i];
            // coefficient 'a' for leaky integrator: z = a*z + b*input
            float a = std::expf(-1.0f / (tau * static_cast<float>(sampleRate_)));
            p.a = a;
            p.b = 1.0f - a;
            p.z = prevCvValue_; // initialize to current held CV
            dielectricPoles_.push_back(p);
        }
    }
    else
    {
        dielectricPoles_.clear();
    }

    setSubProcess(&CyclePeakLookahead::subProcess);
    pinOut_.setStreaming(true);
    pinCV_.setStreaming(true);
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

    // Read controls (block-safe reads)
    float threshold = pinThreshold_.getValue() * 0.1f; // map 0..1 to 0..10V-equivalent
    float ratio = std::clamp(pinRatio_.getValue(), 1.0f, 20.0f);
    float harmonicMix = std::clamp(pinHarmonicMix_.getValue(), 0.0f, 1.0f);

    const int N = lookaheadSamples_;

    // For every sample in block
    for (int s = 0; s < sampleFrames; ++s)
    {
        float x = in[s];
        samplesSinceCycleStart_++;

        // --- Detect positive-going zero crossing (new input cycle) ---
        if (lastSample_ <= 0.0f && x > 0.0f)
        {
            if (samplesSinceCycleStart_ > minCycleGuard_)
            {
                int cycleLength = samplesSinceCycleStart_;
                lastPositiveWidth_ = cycleLength;
                minCycleGuard_ = lastPositiveWidth_ / 4;

                previousCyclePeak_ = cyclePeak_;
                cyclePeak_ = 0.0f;

                // compute next CV target proportional to compressed peak
                nextCvValue_ = 1.0f;
                if (previousCyclePeak_ > 0.0f)
                {
                    float over = std::max(0.0f, previousCyclePeak_ - threshold);
                    float compressed = threshold + over / ratio;
                    nextCvValue_ = std::clamp(compressed / previousCyclePeak_, 0.0f, 1.0f);
                }

                // start after-only half-cosine ramp
                rampSamplesRemaining_ = rampLength_;
                samplesSinceCycleStart_ = 0;
            }
        }

#ifdef FULL_WAVE_PEAK
        float valueForPeak = std::fabs(x);
#else
        float valueForPeak = x;
#endif
        if (valueForPeak > cyclePeak_) cyclePeak_ = valueForPeak;

        lastSample_ = x;

        // write audio sample into lookahead buffer
        lookaheadBuffer_[bufferWritePos_] = x;

        // ---------------- CV: after-ramp or hold ----------------
        float cv_in = prevCvValue_;
        if (rampSamplesRemaining_ > 0)
        {
            // compute half-cosine (Hann) normalized t: 0..1 across rampLength_
            int samplesIntoRamp = rampLength_ - rampSamplesRemaining_;
            float t = static_cast<float>(samplesIntoRamp) / static_cast<float>(rampLength_);
            float w = 0.5f * (1.0f - std::cosf(static_cast<float>(M_PI) * t)); // Hann half-cosine
            cv_in = prevCvValue_ + w * (nextCvValue_ - prevCvValue_);

            --rampSamplesRemaining_;
            if (rampSamplesRemaining_ == 0)
            {
                // commit the immediate target into prevCvValue_ so subsequent blocks start from it
                prevCvValue_ = nextCvValue_;
            }
        }
        else
        {
            // hold prevCvValue_ as-is (dielectric poles will smoothly move it when enabled)
            cv_in = prevCvValue_;
        }

        // ---------------- Dielectric multi-pole smoothing ----------------
        float cv_smooth = cv_in;
        if (!dielectricPoles_.empty())
        {
            // update each pole with cv_in as input (this creates slow tails/DA)
            float accum = 0.0f;
            for (size_t p = 0; p < dielectricPoles_.size(); ++p)
            {
                Pole& pole = dielectricPoles_[p];
                // z = a*z + b*input
                pole.z = pole.a * pole.z + pole.b * cv_in;
                // weighted sum (use default_weights)
                float w = default_weights[p];
                accum += w * pole.z;
            }
            // normalize if sum of weights ~1 -> accum is final smoothed value
            cv_smooth = accum;
            // keep prevCvValue_ updated slowly toward smoothed value so subsequent ramps start from smoothed baseline
            prevCvValue_ = cv_smooth;
        }

        // ---------------- Harmonic shaping (emphasize even and 3rd order) ----------------
        // Use a simple polynomial shaping: y = c1*x + c2*x^2 + c3*x^3
        // We choose coefficients so the linear term dominates, and even+3rd add color.
        // Mix between raw and shaped according to harmonicMix (0..1).
        float shaped = cv_smooth;
        if (harmonicMix > 0.0001f)
        {
            // coefficients tuned empirically: keep unity gain at mid-range roughly
            const float c1 = 0.8f;   // linear
            const float c2 = 0.25f;  // even-harmonic emphasis (2nd)
            const float c3 = -0.05f; // 3rd-order mild
            // compute polynomial on normalized cv_smooth (0..1)
            float x1 = cv_smooth;
            float x2 = x1 * x1;
            float x3 = x2 * x1;
            float poly = c1 * x1 + c2 * x2 + c3 * x3;
            // normalize roughly: scale poly back to 0..1 loosely (since coeffs small)
            // Use mix to blend original and shaped
            shaped = (1.0f - harmonicMix) * cv_smooth + harmonicMix * poly;
        }

        // clamp shaped CV to [0,1]
        float cv_final = std::clamp(shaped, 0.0f, 1.0f);

        // ---------------- Dither ----------------
        if (cachedDitherScale_ > 0.0f)
        {
            cv_final += cachedDitherScale_ * uniDist_(rng_) * 0.5f; // scaled white noise
            // clamp again
            cv_final = std::clamp(cv_final, 0.0f, 1.0f);
        }

        // write CV into cvBuffer_ aligned with the audio bufferWritePos_
        cvBuffer_[bufferWritePos_] = cv_final;

        // --- output delayed audio + CV (both read at same readPos) ---
        int readPos = (bufferWritePos_ + 1) % N;
        out[s] = lookaheadBuffer_[readPos];
        cvOut[s] = cvBuffer_[readPos];

        // advance circular buffer write pointer
        bufferWritePos_ = (bufferWritePos_ + 1) % N;
    }
}

// Register plugin with SE
namespace
{
    int r = Register<CyclePeakLookahead>::withId(L"CyclePeakLookahead_SE");
}
