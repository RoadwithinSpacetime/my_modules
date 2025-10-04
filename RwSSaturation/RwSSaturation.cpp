#include "RwSSaturation.h"
#include <cstring>

#undef max
#undef min

// Build windowed sinc (Hamming window), normalized to DC = 1
std::vector<float> RwSSaturation::make_gsinc_coeffs(int taps, double sampleRate, double cutoffHz)
{
    std::vector<float> h(taps);
    const int mid = (taps - 1) / 2;
    const double fc = cutoffHz / sampleRate;

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
        double w = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / (taps - 1));
        h[n] = static_cast<float>(sinc * w);
    }

    // normalize
    double sum = 0.0;
    for (auto v : h) sum += v;
    if (sum != 0.0)
    {
        for (auto& v : h) v = static_cast<float>(v / sum);
    }
    return h;
}

// FIR convolution using circular buffer
inline float RwSSaturation::fir_filter(const std::vector<float>& buf, int pos, const std::vector<float>& coeffs)
{
    const int N = static_cast<int>(coeffs.size());
    double acc = 0.0;
    int idx = pos;
    for (int k = 0; k < N; ++k)
    {
        acc += coeffs[k] * buf[idx];
        if (--idx < 0) idx += N;
    }
    return static_cast<float>(acc);
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
    initializePin(pinHyst_);
}

int32_t RwSSaturation::open()
{
    sampleRate_ = getSampleRate();
    if (sampleRate_ <= 0.0) sampleRate_ = 44100.0;

    const double cutoffHz = 4000.0;
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

    const float drive = std::clamp(pinDrive_.getValue(), 0.0f, 20.0f);
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alpha = pinAlpha_.getValue(); // even harmonic
    const float beta = pinBeta_.getValue();  // odd harmonic
    const float hyst = std::clamp(pinHyst_.getValue(), 0.0f, 1.0f);

    const int N = gsincTaps_;
    auto& coeff = gsincCoeffs_;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float input = inBuf[s];

        // Pre-gain
        float driven = input * drive;
        driven = std::clamp(driven, -10.0f, 10.0f);

        // Harmonic generators
        float quadSample = driven * std::fabs(driven); // 2nd harmonic generator
        float cubicSample = driven * driven * driven;  // 3rd harmonic generator

        // Write into circular buffers
        quadPos_ = (quadPos_ + 1) % N;
        cubicPos_ = (cubicPos_ + 1) % N;
        quadBuf_[quadPos_] = quadSample;
        cubicBuf_[cubicPos_] = cubicSample;

        // Low-pass filter
        float quadFiltered = fir_filter(quadBuf_, quadPos_, coeff);
        float cubicFiltered = fir_filter(cubicBuf_, cubicPos_, coeff);

        // Scale by alpha/beta
        float quadContribution = alpha * quadFiltered;
        float cubicContribution = beta * cubicFiltered;

        // Combine with linear path
        float combined = driven + quadContribution + cubicContribution;

        // Hysteresis smoothing
        hystState_ += hyst * (combined - hystState_);

        float wet = hystState_;

        // Mix dry and wet
        float out = (1.0f - mix) * input + mix * wet;
        out = std::clamp(out, -20.0f, 20.0f);

        outBuf[s] = out;
    }
}

// Register plugin
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
