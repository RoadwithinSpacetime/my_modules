#pragma once
#include "mp_sdk_audio.h"
#include <vector>
#include <cmath>
#include <algorithm>

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
    FloatInPin  pinDrive_;     // pre-gain
    FloatInPin  pinMix_;       // dry/wet 0..1
    FloatInPin  pinAlpha_;     // alpha -> 2nd harmonic weight (even)
    FloatInPin  pinBeta_;      // beta  -> 3rd harmonic weight (odd)
    FloatInPin  pinThreshold_; // linear amplitude threshold for saturation (0..2)
    FloatInPin  pinKnee_;      // knee width (linear units)
    FloatInPin  pinHyst_;      // hysteresis smoothing 0..1

    // GSinc FIR (9 taps) for LP on harmonic products
    std::vector<float> gsincCoeffs_;
    const int gsincTaps_ = 9;

    // Circular buffers for harmonic products
    std::vector<float> quadBuf_;   // stores x^2 * sign (even generator)
    std::vector<float> cubicBuf_;  // stores x^3
    int quadPos_;
    int cubicPos_;

    // Hysteresis state (one pole)
    float hystState_;

    // sample rate
    double sampleRate_;

    // Helpers
    static std::vector<float> make_gsinc_coeffs(int taps, double sampleRate, double cutoffHz);
    inline float fir_filter(const std::vector<float>& buf, int pos, const std::vector<float>& coeffs);
    inline float smoothstep_cubic(float x); // 0..1 smooth cubic step
};
