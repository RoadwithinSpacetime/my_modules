#include "RwSSaturation.h"
#include <algorithm>

#undef max
#undef min

RwSSaturation::RwSSaturation()
    : firPos_(0)
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

    // FIR design: FIR taps and cutoff
    // Default taps: 31 (good attenuation above cutoff). If CPU is limited, reduce this to 9 or 15.
    firTaps_ = 31;
    const double cutoffHz = 4000.0;
    makeFIR(firTaps_, sampleRate_, cutoffHz, firCoeffs_);

    // allocate circular buffers for x^2 and x^3 paths
    x2Buf_.assign(firTaps_, 0.0f);
    x3Buf_.assign(firTaps_, 0.0f);
    firPos_ = 0;

    // DC removal for x^2: time constant around 50 ms (tweakable)
    const double dcTauSec = 0.05; // 50 ms
    x2_dc_alpha_ = static_cast<float>(1.0 - std::exp(-1.0 / (sampleRate_ * dcTauSec)));
    x2_dc_ = 0.0f;

    // initial hysteresis state
    hystState_ = 0.0f;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);

    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    // nothing dynamic to update besides maybe live parameter mapping; ensure processing active
    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
}

void RwSSaturation::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    if (out)
        std::memset(out, 0, sampleFrames * sizeof(float));

    // wake up if input starts streaming (framework may handle; we keep minimal)
    float* in = getBuffer(pinIn_);
    if (in && pinIn_.isStreaming())
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
        subProcess(sampleFrames);
    }
}

// build a windowed sinc (Hamming) lowpass
void RwSSaturation::makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs)
{
    coeffs.clear();
    coeffs.resize(taps);
    const int M = taps - 1;
    const double fc = cutoffHz / sampleRate; // normalized 0..0.5 (we divide by sampleRate not Nyquist intentionally to match earlier code)
    // but to get proper normalized frequency in terms of pi, use fc = cutoffHz / (sampleRate/2) -> we will use sinc formulation accordingly
    const double fc_norm = cutoffHz / (sampleRate / 2.0); // 0..1 where 1 = Nyquist*2, use in sinc as pi*fc_norm/2? Simpler: use classic sinc with fc/sampleRate
    // we'll implement sinc with fc/sampleRate (safe)
    const double fc_sinc = cutoffHz / sampleRate; // small value like 4000/48000=0.083

    for (int n = 0; n < taps; ++n)
    {
        int k = n - M / 2;
        double h;
        if (k == 0)
        {
            h = 2.0 * fc_sinc; // sinc(0) limit
        }
        else
        {
            double x = 2.0 * M_PI * fc_sinc * k;
            h = std::sin(x) / (M_PI * k);
        }
        double w = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / M); // Hamming
        coeffs[n] = static_cast<float>(h * w);
    }
    // normalize to unity gain at DC
    double sum = 0.0;
    for (auto v : coeffs) sum += v;
    if (sum != 0.0)
    {
        for (auto& v : coeffs) v = static_cast<float>(v / sum);
    }
}

// FIR processing for one input sample (circular buffer)
inline float RwSSaturation::firProcess(std::vector<float>& buf, int pos, const std::vector<float>& coeffs, float input)
{
    buf[pos] = input;
    double acc = 0.0;
    int idx = pos;
    const int N = static_cast<int>(coeffs.size());
    for (int i = 0; i < N; ++i)
    {
        acc += coeffs[i] * buf[idx];
        --idx;
        if (idx < 0) idx += N;
    }
    return static_cast<float>(acc);
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out) return;

    const float drive = std::max(0.0f, pinDrive_.getValue());
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alpha = pinAlpha_.getValue(); // 3rd harmonic control
    const float beta = pinBeta_.getValue();  // 2nd harmonic control
    const float hyst = pinHyst_.getValue();  // hysteresis smoothing 0..1

    // runtime effective scales: combine knob and internal normalization
    const float effBeta = beta * betaScale_;
    const float effAlpha = alpha * alphaScale_;

    const int N = static_cast<int>(firCoeffs_.size());

    for (int s = 0; s < sampleFrames; ++s)
    {
        // linear (unfiltered) path
        float x_lin = in[s] * drive;

        // compute raw nonlinear products from driven signal
        float x2 = x_lin * x_lin;         // positive DC bias present
        float x3 = x_lin * x_lin * x_lin; // odd

        // DC removal for x2 (one-pole lowpass then subtract)
        x2_dc_ += x2_dc_alpha_ * (x2 - x2_dc_);
        float x2_hp = x2 - x2_dc_; // remove slow varying DC component

        // Filter the nonlinear products via shared FIR (we process x2_hp and x3 separately through their buffers)
        // advance circular position
        int pos = firPos_;
        // process x2 path
        float x2f = firProcess(x2Buf_, pos, firCoeffs_, x2_hp);
        // process x3 path using separate buffer but same coeffs
        float x3f = firProcess(x3Buf_, pos, firCoeffs_, x3);

        // advance write position
        firPos_ = (firPos_ + 1) % N;

        // form harmonic contributions
        float harm2 = effBeta * x2f;
        float harm3 = effAlpha * x3f;

        // combine: linear minus/add harmonics (user wanted linear - a*LP(x^3) + B*LP(x^2))
        float shaped = x_lin - harm3 - harm2;

        // hysteresis smoothing (one-pole lowpass on shaped signal)
        hystState_ += hyst * (shaped - hystState_);
        float wet = hystState_;

        // mix dry/wet (dry = original input without drive)
        float outSample = (1.0f - mix) * in[s] + mix * wet;

        // safety clamp
        if (!std::isfinite(outSample)) outSample = 0.0f;
        outSample = std::clamp(outSample, -20.0f, 20.0f);

        out[s] = outSample;
    }
}

// Register plugin
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
