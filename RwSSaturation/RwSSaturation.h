#pragma once
#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace gmpi;

class RwSSaturation : public MpBase2
{
public:
    RwSSaturation();

    int32_t open() override;
    void onSetPins(); // not marked override to match MpBase2 signature
    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatInPin  pinDrive_;  // input drive
    FloatInPin  pinMix_;    // dry/wet mix
    FloatInPin  pinAlpha_;  // 3rd harmonic strength
    FloatInPin  pinBeta_;   // 2nd harmonic strength
    FloatInPin  pinHyst_;   // hysteresis amount

    // FIR (shared for harmonics)
    void makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs);
    float firProcess(std::vector<float>& buf, int pos, const std::vector<float>& coeffs, float input);

    std::vector<float> firCoeffs_;
    std::vector<float> x2Buf_;
    std::vector<float> x3Buf_;
    int firTaps_ = 31;   // default taps (9). Increase to 31 for steeper roll-off.
    int firPos_ = 0;

    // DC removal for even harmonic
    float x2_dc_ = 0.0f;
    float x2_dc_alpha_ = 0.0f;

    // Hysteresis state
    float hystState_ = 0.0f;

    // Scales (tweakable constants)
    const float betaScale_ = 0.25f;  // 2nd harmonic normalization
    const float alphaScale_ = 0.15f; // 3rd harmonic normalization

    double sampleRate_ = 44100.0;
};
