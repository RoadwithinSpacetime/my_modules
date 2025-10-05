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
    void makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs);
    float firProcess(std::vector<float>& buf, int& pos, const std::vector<float>& coeffs, float input);

    // Pins
    AudioInPin pinIn_;
    AudioOutPin pinOut_;
    FloatInPin pinDrive_;
    FloatInPin pinMix_;
    FloatInPin pinAlpha_;
    FloatInPin pinBeta_;
    FloatInPin pinHyst_;
    FloatInPin pinHystFreq_; // external control for hysteresis cutoff frequency

    // FIR and buffers
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

    float sampleRate_;

    // State variables
    float x2_dc_;
    float x2_dc_alpha_;
    float hystState_;

    // scaling factors
    const float alphaScale_ = 1.0f;
    const float betaScale_ = 1.0f;
};
