#include "RwSSaturation.h"
#include <cstring>
#include <algorithm>

#undef max
#undef min  // Fix Windows macros messing with std::max/std::min

RwSSaturation::RwSSaturation()
{
    initializePin(pinIn_);
    initializePin(pinOut_);
    initializePin(pinDrive_);
    initializePin(pinMix_);
    initializePin(pinAlpha_);
    initializePin(pinHyst_);
}

int32_t RwSSaturation::open()
{
    sampleRate_ = getSampleRate();

    // Simple 1-pole LPF at 4 kHz
    double cutoff = 4000.0;
    double x = exp(-2.0 * M_PI * cutoff / sampleRate_);
    lp_a0_ = 1.0f - (float)x;
    lp_b1_ = (float)x;
    lp_z1_ = 0.0f;

    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);

    return MpBase2::open();
}

void RwSSaturation::onSetPins()
{
    setSubProcess(&RwSSaturation::subProcess);
    pinOut_.setStreaming(true);
}

void RwSSaturation::subProcessSilent(int sampleFrames)
{
    float* out = getBuffer(pinOut_);
    if (out)
    {
        memset(out, 0, sampleFrames * sizeof(float));
    }
}

void RwSSaturation::subProcess(int sampleFrames)
{
    const float* in = getBuffer(pinIn_);
    float* out = getBuffer(pinOut_);

    if (!in || !out) return;

    float drive = (std::max)(0.0f, pinDrive_.getValue());
    float mix = (std::clamp)(pinMix_.getValue(), 0.0f, 1.0f);
    float alpha = pinAlpha_.getValue();
    float hyst = pinHyst_.getValue();

    for (int s = 0; s < sampleFrames; ++s)
    {
        // Apply drive
        float x = in[s] * drive;

        // Lowpass filter before cubic
        lp_z1_ = lp_a0_ * x + lp_b1_ * lp_z1_;
        float filtered = lp_z1_;

        // Cubic waveshaper: x + a * x3
        float shaped = filtered + alpha * (filtered * filtered * filtered);

        // Hysteresis smoothing (simple one-pole)
        hystState_ += hyst * (shaped - hystState_);
        float saturated = hystState_;

        // Dry/Wet mix
        out[s] = (1.0f - mix) * in[s] + mix * saturated;
    }
}


// Register with SynthEdit
namespace
{
    auto r = Register<RwSSaturation>::withId(L"RwSSaturation");
}
