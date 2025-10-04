#pragma once
#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace gmpi;

class RwSSaturation : public MpBase2
{
public:
    RwSSaturation();

    int32_t open() override; // this one is correct
    void onSetPins();        // remove 'override'
    void subProcess(int sampleFrames); // remove 'override'
    void subProcessSilent(int sampleFrames); // remove 'override'

private:
    // Pins
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    FloatInPin  pinDrive_;
    FloatInPin  pinMix_;
    FloatInPin  pinAlpha_;
    FloatInPin  pinBeta_;
    FloatInPin  pinHyst_;

    // FIR filter (shared for harmonics)
    std::vector<float> firCoeffs_;
    std::vector<float> x2Buf_;
    std::vector<float> x3Buf_;
    int firPos_ = 0;
    int firTaps_ = 31;

    // DC removal
    float x2_dc_ = 0.0f;
    float x2_dc_alpha_ = 0.0f;

    // Hysteresis
    float hystState_ = 0.0f;

    // Scaling constants
    float betaScale_ = 0.5f;
    float alphaScale_ = 0.15f;

    double sampleRate_ = 44100.0;

    // Helpers
    void makeFIR(int taps, double sampleRate, double cutoffHz, std::vector<float>& coeffs);
    inline float firProcess(std::vector<float>& buf, int pos, const std::vector<float>& coeffs, float input);
};
