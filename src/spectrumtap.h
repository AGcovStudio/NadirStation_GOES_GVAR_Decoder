#pragma once
// Spectrum frames for the display at a fixed cadence in stream time.
//
// The receivers used to compute one spectrum per 100 ms of wall clock, from the last FFT-length
// samples of whatever block had just arrived, which made the waterfall advance ten rows a second
// and tied its speed to the driver's block size (an RTL-SDR block is 55 ms). This takes a frame
// every `interval` seconds of signal, from inside the block where that instant falls, so the
// waterfall runs at the same speed for every device and block size.
#include "dsp.h"
#include <algorithm>
#include <complex>
#include <cstddef>
#include <vector>

namespace nadirstation::dsp {

class SpectrumTap {
public:
    explicit SpectrumTap(double intervalSeconds = 0.02) : interval_(intervalSeconds) {}
    void reset() { untilNext_ = 0.0; }

    // Calls sink(const SpectrumResult&) for every frame due inside this block. (The parameter must
    // not be called `emit`: Qt defines that as an empty macro, which silently turned the call into
    // a discarded expression and left the Pluto and RTL-SDR spectrum without a single frame.)
    template <typename Sink>
    void feed(const std::vector<std::complex<float>>& block, double sampleRate, std::size_t fftSize,
              SpectrumWindow window, Sink&& sink) {
        if (block.empty() || sampleRate <= 0.0 || fftSize < 16) return;
        const double step = std::max(1.0, interval_ * sampleRate);
        double pos = untilNext_;
        std::vector<std::complex<float>> frame(fftSize);
        while (pos < static_cast<double>(block.size())) {
            const std::size_t end = static_cast<std::size_t>(pos) + 1;   // frame ends at this sample
            if (end >= fftSize && end <= block.size()) {
                std::copy(block.begin() + static_cast<std::ptrdiff_t>(end - fftSize),
                          block.begin() + static_cast<std::ptrdiff_t>(end), frame.begin());
                sink(spectrum(frame, static_cast<float>(sampleRate), fftSize, window));
            }
            pos += step;
        }
        untilNext_ = pos - static_cast<double>(block.size());
    }

private:
    double interval_;
    double untilNext_ = 0.0;
};

}  // namespace nadirstation::dsp
