#include "RwSSaturation.h"
#include <algorithm>

#undef max
#undef min

RwSSaturation::RwSSaturation()
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

    firTaps_ = 9;
    makeFIR(firTaps_, sampleRate_, 4000.0, firCoeffs_);
    x2Buf_.assign(firTaps_, 0.0f);
    x3Buf_.assign(firTaps_, 0.0f);
    firPos_ = 0;

    const double dcTau = 0.05;
    x2_dc_alpha_ = static_cast<float>(1.0 - std::exp(-1.0 / (sampleRate_ * dcTau)));
    x2_dc_ = 0.0f;

    hystState_ = 0.0f;
    active_ = true;
    silentCounter_ = 0;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    if (!active_)
    {
        setSubProcess(&RwSSaturation::subProcess);
        pinOut_.setStreaming(true);
        active_ = true;
    }
}

void RwSSaturation::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    if (out)
        std::memset(out, 0, sampleFrames * sizeof(float));

    // Check for signal to wake up
    const float* in = getBuffer(pinIn_);
    if (in && pinIn_.isStreaming())
    {
        for (int i = 0; i < sampleFrames; ++i)
        {
            if (std::fabs(in[i]) > 1e-8f)
            {
                setSubProcess(&RwSSaturation::subProcess);
                pinOut_.setStreaming(true);
                active_ = true;
                return;
            }
        }
    }
}

void RwSSaturation::makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs)
{
    coeffs.resize(taps);
    const int M = taps - 1;
    const double fc = cutoffHz / sampleRate;

    for (int n = 0; n < taps; ++n)
    {
        int k = n - M / 2;
        double h = (k == 0) ? 2.0 * fc : std::sin(2.0 * M_PI * fc * k) / (M_PI * k);
        double w = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / M);
        coeffs[n] = static_cast<float>(h * w);
    }

    double sum = 0.0;
    for (auto c : coeffs) sum += c;
    for (auto& c : coeffs) c = static_cast<float>(c / sum);
}

float RwSSaturation::firProcess(std::vector<float>& buf, int pos, const std::vector<float>& coeffs, float input)
{
    buf[pos] = input;
    double acc = 0.0;
    int idx = pos;
    const int N = static_cast<int>(coeffs.size());
    for (int i = 0; i < N; ++i)
    {
        acc += coeffs[i] * buf[idx];
        if (--idx < 0) idx = N - 1;
    }
    return static_cast<float>(acc);
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);
    if (!in || !out)
        return;

    float driveParam = std::max(0.0001f, pinDrive_.getValue());
    float knee = driveKnee_;
    float drive = 1.0f + (driveParam - 1.0f) * (1.0f - std::exp(-knee * std::fabs(driveParam - 1.0f)));

    const float trim = 1.0f / drive; // mathematical inverse trim
    const float mix = std::clamp(pinMix_.getValue(), 0.0f, 1.0f);
    const float alpha = pinAlpha_.getValue();
    const float beta = pinBeta_.getValue();
    const float hyst = pinHyst_.getValue();

    const float effBeta = beta * betaScale_;
    const float effAlpha = alpha * alphaScale_;
    const int N = static_cast<int>(firCoeffs_.size());

    bool silentBlock = true;

    for (int s = 0; s < sampleFrames; ++s)
    {
        float x_in = in[s];
        if (std::fabs(x_in) > 1e-8f)
            silentBlock = false;

        // linear drive
        float x_lin = x_in * drive;

        // nonlinear products
        float x2 = x_lin * x_lin;
        float x3 = x_lin * x_lin * x_lin;

        // DC remove even term
        x2_dc_ += x2_dc_alpha_ * (x2 - x2_dc_);
        float x2_hp = x2 - x2_dc_;

        int pos = firPos_;
        float x2f = firProcess(x2Buf_, pos, firCoeffs_, x2_hp);
        float x3f = firProcess(x3Buf_, pos, firCoeffs_, x3);
        firPos_ = (firPos_ + 1) % N;

        float shaped = x_lin - effBeta * x3f + effAlpha * x2f;

        hystState_ += hyst * (shaped - hystState_);
        float wet = hystState_ * trim;

        float outSample = (1.0f - mix) * x_in + mix * wet;

        outSample = std::clamp(outSample, -20.0f, 20.0f);
        out[s] = outSample;
    }

    // Auto-sleep detection
    if (silentBlock)
    {
        silentCounter_ += sampleFrames;
        if (silentCounter_ > kSilentFramesBeforeSleep)
        {
            setSubProcess(&RwSSaturation::subProcessSilent);
            active_ = false;
        }
    }
    else
    {
        silentCounter_ = 0;
    }
}

// Register
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
