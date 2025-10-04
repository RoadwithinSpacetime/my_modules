#include "RwSSaturation.h"
#include <cstring>   // memset
#include <algorithm> // std::clamp, std::max
#include <numeric>   // accumulate

#undef max
#undef min

// Helper: compute normalized windowed sinc coeffs (odd taps)
static std::vector<float> make_gsinc_coeffs(int taps, double sampleRate, double cutoffHz)
{
    std::vector<float> h(taps);
    const int mid = (taps - 1) / 2;
    const double fc = cutoffHz / sampleRate; // normalized freq (0..0.5)
    // Hamming window
    for (int n = 0; n < taps; ++n)
    {
        int k = n - mid;
        double x = (k == 0) ? 1.0 : std::sin(2.0 * M_PI * fc * k) / (M_PI * k);
        // Hamming window
        double w = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / (taps - 1));
        h[n] = static_cast<float>(x * w);
    }
    // Normalize to unity gain at DC
    double sum = 0.0;
    for (auto& v : h) sum += v;
    if (sum != 0.0)
    {
        for (auto& v : h) v = static_cast<float>(v / sum);
    }
    return h;
}

RwSSaturation::RwSSaturation()
    : cubicBufPos_(0)
    , hystState_(0.0f)
    , sampleRate_(44100.0)
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
    initializePin(pinAlpha_);
    initializePin(pinHyst_);
    initializePin(pinThreshold_);
}

int32_t RwSSaturation::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.0) sampleRate_ = 44100.0;

    // default 4 kHz cutoff for cubic harmonic smoothing
    const double cutoffHz = 4000.0;

    gsincCoeffs_ = make_gsinc_coeffs(gsincTaps_, sampleRate_, cutoffHz);
    cubicBuf_.assign(gsincTaps_, 0.0f);
    cubicBufPos_ = 0;

    // Reset states
    hystState_ = 0.0f;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);

    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    // Ensure streaming state follows input
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
    // When input not streaming, output silence
    float* out = getBuffer(pinOut_);
    if (out) memset(out, 0, sampleFrames * sizeof(float));

    // But wake up if input starts streaming in this block
    float* in = getBuffer(pinIn_);
    if (in && pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
        // Immediately process rest of block using subProcess
        subProcess(sampleFrames);
    }
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* inBuf = getBuffer(pinIn_);
    float* outBuf = getBuffer(pinOut_);
    if (!inBuf || !outBuf) return;

    // Read controls safely
    const float drive = std::clamp(pinDrive_.getValue(), 0.0f, 20.0f); // allow up to 20x
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alphaBase = pinAlpha_.getValue(); // e.g. 0..2 typical
    const float hyst = std::clamp(pinHyst_.getValue(), 0.0f, 1.0f); // smoothing factor
    const float threshold = std::clamp(pinThreshold_.getValue(), 0.0f, 2.0f); // linear threshold

    // alpha mapping: optionally link alpha to drive for more aggressive curve at higher drive
    const float alpha = alphaBase * drive;

    const int N = gsincTaps_;
    auto& coeff = gsincCoeffs_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float input = inBuf[s];

        // Pre-gain (drive)
        float driven = input * drive;

        // Bound driven to avoid extreme overflow in cubic
        if (!std::isfinite(driven)) driven = 0.0f;
        driven = std::clamp(driven, -10.0f, 10.0f);

        // Compute cubic sample (cubic path)
        float cubicSample = driven * driven * driven; // x^3

        // Insert cubic sample into circular buffer for FIR
        cubicBufPos_ = (cubicBufPos_ + 1) % N;
        cubicBuf_[cubicBufPos_] = cubicSample;

        // Convolve gsinc FIR (9 taps) -> filtered cubic
        double filteredCubic = 0.0;
        int idx = cubicBufPos_;
        for (int k = 0; k < N; ++k)
        {
            filteredCubic += coeff[k] * cubicBuf_[idx];
            idx = (idx == 0) ? (N - 1) : (idx - 1);
        }
        float filtered = static_cast<float>(filteredCubic);

        // Scale cubic term by alpha * activation mask based on threshold:
        // activation = clamp( (|driven| - threshold) / (maxDrive - threshold), 0..1 )
        // choose maxDrive reference to 1.0 (or use drive). We'll use 1.0 as nominal full-scale.
        float absDriven = std::fabs(driven);
        float activation = 0.0f;
        if (absDriven > threshold)
        {
            // scale to 0..1 relative to (threshold .. saturationRange)
            // choose saturationRange = 1.0 + small headroom to allow scaling.
            const float saturationRange = 1.0f + 0.01f;
            activation = (absDriven - threshold) / (saturationRange - threshold);
            activation = std::clamp(activation, 0.0f, 1.0f);
        }

        float cubicContribution = alpha * activation * filtered;

        // Linear path is the driven signal (we include the linear pre-gain)
        float linearPath = driven;

        // Combine: x + alpha * LP(x^3)
        float shaped = linearPath + cubicContribution;

        // Hysteresis smoothing (one-pole)
        hystState_ += hyst * (shaped - hystState_);
        if (!std::isfinite(hystState_)) hystState_ = 0.0f;

        float outWet = hystState_;

        // Mix with original input (dry) using mix
        float out = (1.0f - mix) * input + mix * outWet;

        // Safety clamp output to reasonable audio range
        if (!std::isfinite(out)) out = 0.0f;
        out = std::clamp(out, -20.0f, 20.0f); // very large clamps to be safe

        outBuf[s] = out;
    }
}

// Register plugin with SE
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
