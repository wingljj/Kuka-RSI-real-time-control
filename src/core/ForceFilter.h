#pragma once

// Second-order Butterworth low-pass filter.
// Design: bilinear transform from analog prototype H(s) = 1 / (s² + √2·s + 1),
// with frequency prewarping (c = 1/tan(ω0·T/2)).
// Coefficients precomputed on configure(). step() is 4 mul + 3 add, no allocation.
class Butterworth2
{
public:
    Butterworth2() = default;

    // fc: cutoff frequency (Hz), must be > 0 and < fs/2
    // fs: sample rate (Hz)
    // Invalid fc/fs (≤0, or fc ≥ fs/2) falls back to pass-through (b0=1):
    // bilinear prewarping gives c ≤ 0 outside fc ∈ (0, fs/2), which moves the
    // poles out of the unit circle — an unstable filter must never be produced.
    void configure(double fc, double fs);

    // Process one sample. Returns filtered output.
    double step(double x);

    // Reset filter state. If initialValue is provided, pre-fill state so
    // first step() output ≈ initialValue (no step-response transient).
    void reset(double initialValue = 0.0);

private:
    double b0 = 1.0, b1 = 0.0, b2 = 0.0;
    double a1 = 0.0, a2 = 0.0;
    double x1 = 0.0, x2 = 0.0;  // delayed inputs
    double y1 = 0.0, y2 = 0.0;  // delayed outputs
};
