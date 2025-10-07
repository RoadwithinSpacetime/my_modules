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
    FloatInPin pinHystFreq_;

    // Buffers and states
    std::vector<float> harmFirCoeffs_;
    std::vector<float> harmFirBuf1_;
    std::vector<float> harmFirBuf2_;
    std::vector<float> hystFirCoeffs_;
    std::vector<float> hystFirBuf_;
    std::vector<float> dryDelayBuf_;

    int harmFirPos1_;
    int harmFirPos2_;
    int hystFirPos_;
    int dryDelayPos_;

    int harmFirTaps_;
    int hystFirTaps_;
    int totalLatency_;

    double sampleRate_ = 44100.0;

    // States
    float x2_dc_ = 0.0f;
    float x2_dc_alpha_ = 0.0f;
    float hystState_ = 0.0f;

    const float alphaScale_ = 0.1f;
    const float betaScale_ = 0.25f;
};
