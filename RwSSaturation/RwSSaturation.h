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

private:
    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);

    // FIR helper functions
    void makeFIR(int taps, double sampleRate, float cutoffHz, std::vector<float>& coeffs);
    float firProcess(std::vector<float>& buf, int& pos, const std::vector<float>& coeffs, float input);

    // Pins
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    FloatInPin pinDrive_;
    FloatInPin pinMix_;
    FloatInPin pinAlpha_;
    FloatInPin pinBeta_;
    FloatInPin pinHyst_;
    FloatInPin pinHystFreq_;

    // FIR buffers & coefficients
    std::vector<float> harmFirCoeffs_;
    std::vector<float> harmFirBuf1_;
    std::vector<float> harmFirBuf2_;
    std::vector<float> hystFirCoeffs_;
    std::vector<float> hystFirBuf_;

    int harmFirPos1_;
    int harmFirPos2_;
    int hystFirPos_;

    int harmFirTaps_;
    int hystFirTaps_;

    double sampleRate_ = 44100.0;

    // State
    float x2_dc_ = 0.0f;
    float x2_dc_alpha_ = 0.0f;
    float hystState_ = 0.0f;

    // Harmonic scales
    const float alphaScale_ = 0.1f;  // scales 2nd harmonic
    const float betaScale_ = 0.25f; // scales 3rd harmonic
};
