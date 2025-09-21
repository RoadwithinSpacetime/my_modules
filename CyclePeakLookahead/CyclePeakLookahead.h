#pragma once
#include "mp_sdk_audio.h"
#include <cmath>
#include <vector>
#include <random>

using namespace gmpi;

class CyclePeakLookahead : public MpBase2
{
public:
    CyclePeakLookahead();

    int32_t open() override;
    void onSetPins() override;
    void subProcess(int sampleFrames);
    void subProcessSilent(int sampleFrames);

private:
    // ---------- Pins ----------
    AudioInPin  pinIn_;
    AudioOutPin pinOut_;
    AudioOutPin pinCV_;            // CV out (normalized 0..1; SE maps to 0..10V)
    FloatInPin  pinThreshold_;     // 0..1 mapped to 0..10V
    FloatInPin  pinRatio_;         // 1..20 (compressor ratio)
    FloatInPin  pinRampLength_;    // ramp length in samples (after-zero-cross)
    FloatInPin  pinDitherDb_;      // dither level in dB (e.g. -90)
    BoolInPin   pinEnableDielectric_; // enable multi-pole dielectric smoothing
    FloatInPin  pinHarmonicMix_;   // 0..1 mix for harmonic shaping (0=off,1=full shaped)

    // -------- Buffers ----------
    std::vector<float> lookaheadBuffer_;
    std::vector<float> cvBuffer_;
    int bufferWritePos_;
    int lookaheadSamples_;

    // ----- Cycle / peak tracking -----
    float lastSample_;
    float cyclePeak_;
    float previousCyclePeak_;
    int samplesSinceCycleStart_;
    int lastPositiveWidth_;
    int minCycleGuard_;

    // ----- CV ramp & state -----
    int rampLength_;                // after-only ramp samples
    float prevCvValue_;             // current held CV (0..1)
    float nextCvValue_;             // computed at zero-cross
    int rampSamplesRemaining_;      // countdown after zero-cross

    // ----- Dielectric (multi-pole) -----
    struct Pole { float a; float b; float z; };
    std::vector<Pole> dielectricPoles_; // per-pole state

    // ----- Misc -----
    double sampleRate_;

    // Dither generator
    std::mt19937 rng_;
    std::uniform_real_distribution<float> uniDist_;

    // Parameters cached per-block
    float cachedDitherScale_;  // linear amplitude from dB
};
