#pragma once
#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace gmpi;

class RwSSaturation : public MpBase2
{
public:
    RwSSaturation();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatInPin  pinDrive_;
    FloatInPin  pinMix_;
    FloatInPin  pinAlpha_;    // 2nd harmonic amount
    FloatInPin  pinBeta_;     // 3rd harmonic amount
    FloatInPin  pinHyst_;     // hysteresis strength
    FloatInPin  pinHystFreq_; // hysteresis filter cutoff

    // FIR filter for harmonics
    std::vector<float> firCoeffs_;
    std::vector<float> x2Buf_;
    std::vector<float> x3Buf_;
    int firTaps_ = 31;
    int firPos_ = 0;

    // Hysteresis FIR (separate)
    std::vector<float> hystCoeffs_;
    std::vector<float> hystBuf_;
    std::vector<float> hystCoeffsNew_;
    int hystTaps_ = 31;
    int hystPos_ = 0;

    // FIR helpers
    void makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs);
    float firProcess(std::vector<float>& buf, int pos, const std::vector<float>& coeffs, float input);

    // DC removal for x² path
    float x2_dc_ = 0.0f;
    float x2_dc_alpha_ = 0.0f;

    // Hysteresis state
    float hystState_ = 0.0f;

    // Morphing FIR state
    bool morphing_ = false;
    int morphCounter_ = 0;
    int morphSamples_ = 512;
    float hystCutoffHz_ = 4000.0f;

    // Auto-sleep
    int silenceCounter_ = 0;
    const int silenceThreshold_ = 512; // samples
    bool isSilent_ = false;

    // Internal constants
    double sampleRate_ = 44100.0;
    const float alphaScale_ = 0.25f;
    const float betaScale_ = 0.15f;
};
