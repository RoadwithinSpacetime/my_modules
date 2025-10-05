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
    void onSetPins();
    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatInPin  pinDrive_;
    FloatInPin  pinMix_;
    FloatInPin  pinAlpha_;
    FloatInPin  pinBeta_;
    FloatInPin  pinHyst_;

    // FIR (shared for harmonics)
    void makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs);
    float firProcess(std::vector<float>& buf, int pos, const std::vector<float>& coeffs, float input);

    std::vector<float> firCoeffs_;
    std::vector<float> x2Buf_;
    std::vector<float> x3Buf_;
    int firTaps_ = 9;
    int firPos_ = 0;

    // DC removal
    float x2_dc_ = 0.0f;
    float x2_dc_alpha_ = 0.0f;

    // Hysteresis
    float hystState_ = 0.0f;

    // Scales
    const float betaScale_ = 0.25f;
    const float alphaScale_ = 0.15f;

    // Soft knee curve
    float driveKnee_ = 2.0f; // larger = softer

    // Auto sleep
    bool active_ = true;
    int silentCounter_ = 0;
    static constexpr int kSilentFramesBeforeSleep = 2048;

    double sampleRate_ = 44100.0;
};
