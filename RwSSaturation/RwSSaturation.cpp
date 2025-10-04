#include "RwSSaturation.h"
#include <cstring>   // memset
#include <numeric>   // accumulate
#include <algorithm>

#undef max
#undef min

// Build windowed sinc (Hamming window) normalized to unity DC.
// taps must be odd (9).
std::vector<float> RwSSaturation::make_gsinc_coeffs(int taps, double sampleRate, double cutoffHz)
{
    std::vector<float> h(taps);
    const int mid = (taps - 1) / 2;
    const double fc = cutoffHz / sampleRate; // normalized frequency (0..0.5)
    for (int n = 0; n < taps; ++n)
    {
        int k = n - mid;
        double sinc;
        if (k == 0)
            sinc = 2.0 * fc;
        else
        {
            double x = 2.0 * M_PI * fc * k;
            sinc = std::sin(x) / (M_PI * k);
        }
        // Hamming window
        double w = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / (taps - 1));
        h[n] = static_cast<float>(sinc * w);
    }
    // normalize DC gain = 1
    double sum = 0.0;
    for (auto v : h) sum += v;
    if (sum != 0.0)
    {
        for (auto& v : h) v = static_cast<float>(v / sum);
    }
    return h;
}

// circular-buffer FIR convolution (coeffs length == buffer length)
inline float RwSSaturation::fir_filter(const std::vector<float>& buf, int pos, const std::vector<float>& coeffs)
{
    const int N = static_cast<int>(coeffs.size());
    double acc = 0.0;
    int idx = pos;
    for (int k = 0; k < N; ++k)
    {
        acc += coeffs[k] * buf[idx];
        // move backward circularly
        if (--idx < 0) idx += N;
    }
    return static_cast<float>(acc);
}

// Smooth cubic step smoothstep: 0..1 mapping with zero derivatives at ends.
inline float RwSSaturation::smoothstep_cubic(float x)
{
    // clamp 0..1
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    // 3t^2 - 2t^3
    return x * x * (3.0f - 2.0f * x);
}

RwSSaturation::RwSSaturation()
    : quadPos_(0)
    , cubicPos_(0)
    , hystState_(0.0f)
    , sampleRate_(44100.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
    initializePin(pinAlpha_);
    initializePin(pinBeta_);
    initializePin(pinThreshold_);
    initializePin(pinKnee_);
    initializePin(pinHyst_);
}

int32_t RwSSaturation::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.0) sampleRate_ = 44100.0;

    const double cutoffHz = 4000.0; // LP cutoff for harmonic products
    gsincCoeffs_ = make_gsinc_coeffs(gsincTaps_, sampleRate_, cutoffHz);

    quadBuf_.assign(gsincTaps_, 0.0f);
    cubicBuf_.assign(gsincTaps_, 0.0f);
    quadPos_ = cubicPos_ = 0;

    hystState_ = 0.0f;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);

    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    if (pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
    }
    else
    {
        setSubProcess(&RwSSaturation::subProcessSilent);
        pinOut_.setStreaming(false);
    }
}

void RwSSaturation::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    if (out) memset(out, 0, sampleFrames * sizeof(float));

    // wake if audio reappears in same block
    float* in = getBuffer(pinIn_);
    if (in && pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
        subProcess(sampleFrames);
    }
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* inBuf = getBuffer(pinIn_);
    float* outBuf = getBuffer(pinOut_);
    if (!inBuf || !outBuf) return;

    // Read controls
    const float drive = std::clamp(pinDrive_.getValue(), 0.0f, 20.0f);
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alphaBase = pinAlpha_.getValue(); // weight for 2nd-order (even)
    const float betaBase = pinBeta_.getValue();  // weight for 3rd-order (odd)
    const float threshold = std::clamp(pinThreshold_.getValue(), 0.0f, 5.0f); // linear threshold
    const float knee = std::max(0.0001f, pinKnee_.getValue()); // knee width (avoid 0)
    const float hyst = std::clamp(pinHyst_.getValue(), 0.0f, 1.0f);

    const int N = gsincTaps_;
    auto& coeff = gsincCoeffs_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float input = inBuf[s];

        // Pre-gain (drive)
        float driven = input * drive;
        if (!std::isfinite(driven)) driven = 0.0f;
        // clamp to avoid blow-ups
        driven = std::clamp(driven, -10.0f, 10.0f);

        // --- Generate harmonic products (before filtering) ---
        // Even/2nd generator: x * |x| gives second-harmonic rich term
        float quadSample = driven * std::fabs(driven); // signed squared
        float cubicSample = driven * driven * driven;   // x^3

        // write into circular buffers
        quadPos_ = (quadPos_ + 1) % N;
        cubicPos_ = (cubicPos_ + 1) % N;
        quadBuf_[quadPos_] = quadSample;
        cubicBuf_[cubicPos_] = cubicSample;

        // Apply GSinc FIR to both harmonic products
        float quadFiltered = fir_filter(quadBuf_, quadPos_, coeff);
        float cubicFiltered = fir_filter(cubicBuf_, cubicPos_, coeff);

        // --- Activation / soft threshold (smooth knee) ---
        // map abs(driven) into activation 0..1 using smoothstep with knee
        float absDriven = std::fabs(driven);
        float act = 0.0f;
        if (absDriven <= threshold)
        {
            // below threshold: activation rises smoothly inside knee range
            // compute t = 1 - ((threshold - abs)/knee) clamped
            float t = (threshold - absDriven) / knee;
            // we want small activation near threshold - use reversed smoothstep
            // but simpler: when abs <= threshold -> act = 0
            act = 0.0f;
        }
        else
        {
            // above threshold -> scale upto 1 over small range (knee)
            float t = (absDriven - threshold) / knee; // how far into saturation
            // use smoothstep cubic for gentle ramp
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            act = smoothstep_cubic(t);
        }

        // Alternatively, we want some subtle contribution even below threshold (soft behavior)
        // Implement a soft base contribution that is small even below threshold:
        // baseFactor = clamp(absDriven / threshold, 0..1)   (optional)
        float baseFactor = (threshold > 0.0f) ? std::clamp(absDriven / threshold, 0.0f, 1.0f) : 1.0f;
        // final activation for harmonic paths = mix of baseFactor and act
        float harmonicGain = baseFactor * 0.4f + act * 0.6f; // blend: 40% soft base, 60% threshold-driven

        // --- Scale harmonic contributions by alpha/beta and activation ---
        float alpha = alphaBase; // 2nd harmonic weight (user)
        float beta = betaBase;  // 3rd harmonic weight (user)

        float quadContribution = alpha * harmonicGain * quadFiltered;
        float cubicContribution = beta * harmonicGain * cubicFiltered;

        // Linear path stays as driven (pre-gain)
        float linearPath = driven;

        // Combine: linear + alpha*LP(2nd) + beta*LP(3rd)
        float combined = linearPath + quadContribution + cubicContribution;

        // Hysteresis smoothing (one-pole)
        hystState_ += hyst * (combined - hystState_);
        if (!std::isfinite(hystState_)) hystState_ = 0.0f;

        float wet = hystState_;

        // Mix dry/wet (dry = original input, not pre-gain; wet = processed)
        float out = (1.0f - mix) * input + mix * wet;

        // safety clamp
        if (!std::isfinite(out)) out = 0.0f;
        out = std::clamp(out, -20.0f, 20.0f);

        outBuf[s] = out;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
