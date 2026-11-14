// Soft-decision recovery of the pixel values in an S-VISSR sector that failed its CRC.
//
// The link carries no error correction, so a sector with even one wrong bit fails its CRC and the
// hard decisions go on into the image as they are: at low signal-to-noise ratio that is salt-and-
// pepper noise, and the later impulse repair only partly hides it. What the demodulator also has
// is how sure it was about every symbol, and what the image is known to look like: neighbouring
// pixels along a scan line are close in value. This module combines the two.
//
// The model, per sector:
//   * Scrambling and differential coding are undone first (by the caller), so what arrives is the
//     running parity y of the information bits, y_n = y_(n-1) xor b_n, seen through Gaussian noise.
//     A symbol therefore constrains y directly; working on y rather than on the decoded bits b
//     keeps the fact that one symbol error corrupts two adjacent bits instead of treating the bits
//     as independent, which at 10 to 20 per cent symbol errors is a large part of the difference.
//   * A pixel is B bits, MSB first. Its value v drives the parity through its bits, so the state
//     carried from pixel to pixel is (v, parity at the pixel's start).
//   * Consecutive pixels are related by a prior T(v'|v) = (1-e) * Laplace(v'-v) + e / S: a short
//     tail around the previous value plus a uniform floor for edges.
// A forward-backward pass over the line gives each pixel's posterior; the output is its mean or its
// maximum. Nothing here depends on Qt.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <bit>
#include <cstdint>
#include <vector>
#if defined(__SSE__) || defined(__x86_64__) || defined(_M_X64)
#include <xmmintrin.h>
#define NADIR_SOFTPIXEL_FTZ 1
#endif

namespace nadirstation::softpixel {

struct Prior {
    double scale = 1.8;          // Laplace scale of the step between neighbouring pixels, in levels
    double edgeFraction = 0.01;  // mass spread uniformly over all values, for edges
};

// Optional prior from the row above (the neighbouring detector of the same channel, already decoded): pixel i
// is pulled towards above[i] (full-resolution value, `bits` wide) by (1-edge) * Laplace(scale) + edge.
struct Vertical {
    const uint16_t* above = nullptr;
    double scale = 1.5;          // in chain levels
    double edgeFraction = 0.15;
};

struct Result {
    std::vector<uint16_t> mean;      // posterior mean, rounded
    std::vector<uint16_t> best;      // posterior maximum
    std::vector<float> confidence;   // posterior probability of `best`
};

namespace detail {

// The forward-backward products underflow into denormal floats in the unlikely corners of the state space, and
// arithmetic on denormals is 10-100x slower on x86. They carry no probability worth keeping, so flush them to zero
// for the duration of a decode (restored afterwards; the caller's floating-point mode is not changed).
struct FlushDenormals {
#ifdef NADIR_SOFTPIXEL_FTZ
    unsigned int saved;
    FlushDenormals() : saved(_mm_getcsr()) { _mm_setcsr(saved | 0x8040u); }   // FTZ | DAZ
    ~FlushDenormals() { _mm_setcsr(saved); }
#endif
};

// out[v] = (1-e) * sum_u w(v-u) * in[u] + e * sum(in) / S for the two-sided exponential
// w(d) = (1-a)/(1+a) * a^|d|, a = exp(-1/scale). The exponential is applied as two recursive passes,
// so the cost does not depend on how long the tails are.
inline void applyPrior(double scale, double edgeFraction, int states, const float* in, float* out, float* work) {
    const float a = static_cast<float>(std::exp(-1.0 / std::max(0.05, scale)));
    const float norm = (1.0f - a) / (1.0f + a);
    const float edge = static_cast<float>(edgeFraction);
    float total = 0.0f;
    for (int v = 0; v < states; ++v) total += in[v];
    const float floorMass = edge * total / static_cast<float>(states);
    const float keep = (1.0f - edge) * norm;
    // forward pass into work, backward pass folded into out
    float run = 0.0f;
    for (int v = 0; v < states; ++v) { run = in[v] + a * run; work[v] = run; }
    run = 0.0f;
    for (int v = states - 1; v >= 0; --v) {
        run = in[v] + a * run;
        out[v] = keep * (work[v] + run - in[v]) + floorMass;
    }
}

// Two independent applyPrior calls run in one loop. Each recursion is a chain of dependent
// multiply-adds, so the two chains overlap in the processor instead of waiting on each other.
inline void applyPrior2(double scale, double edgeFraction, int states, const float* in0, float* out0, float* work0,
                        const float* in1, float* out1, float* work1) {
    const float a = static_cast<float>(std::exp(-1.0 / std::max(0.05, scale)));
    const float norm = (1.0f - a) / (1.0f + a);
    const float edge = static_cast<float>(edgeFraction);
    float t0 = 0.0f, t1 = 0.0f;
    for (int v = 0; v < states; ++v) { t0 += in0[v]; t1 += in1[v]; }
    const float floor0 = edge * t0 / static_cast<float>(states), floor1 = edge * t1 / static_cast<float>(states);
    const float keep = (1.0f - edge) * norm;
    float r0 = 0.0f, r1 = 0.0f;
    for (int v = 0; v < states; ++v) { r0 = in0[v] + a * r0; r1 = in1[v] + a * r1; work0[v] = r0; work1[v] = r1; }
    r0 = r1 = 0.0f;
    for (int v = states - 1; v >= 0; --v) {
        r0 = in0[v] + a * r0;
        r1 = in1[v] + a * r1;
        out0[v] = keep * (work0[v] + r0 - in0[v]) + floor0;
        out1[v] = keep * (work1[v] + r1 - in1[v]) + floor1;
    }
}

}  // namespace detail

// llr[n]: ln P(y_n = 0) / P(y_n = 1) for the n-th symbol of the sector data (pixels * bits symbols).
// startParity: y before the first data bit (0 or 1), or -1 when unknown.
// chainBits: how many of the leading bits of each pixel are modelled (the state has 2^chainBits
// values); the remaining bits of the pixel are summed out, which they need to be because they still
// move the parity.
inline Result decodeChain(const float* llr, int pixels, int bits, int chainBits, int startParity,
                          const Prior& prior, const Vertical& vertical = Vertical()) {
    const detail::FlushDenormals ftz;
    Result out;
    out.mean.assign(static_cast<std::size_t>(pixels), 0);
    out.best.assign(static_cast<std::size_t>(pixels), 0);
    out.confidence.assign(static_cast<std::size_t>(pixels), 0.0f);
    if (pixels <= 0 || bits <= 0 || chainBits <= 0 || chainBits > bits) return out;

    const int S = 1 << chainBits;
    const int extra = bits - chainBits;        // bits summed out
    const int X = 1 << extra;
    const std::size_t plane = static_cast<std::size_t>(S);

    // chainParity[v]: parity of all chain bits of v.
    std::vector<uint8_t> chainParity(plane);
    for (int v = 0; v < S; ++v) chainParity[static_cast<std::size_t>(v)] = static_cast<uint8_t>(std::popcount(static_cast<unsigned>(v)) & 1);

    // Per pixel: weight[s][e][v] = how well the received symbols fit value v when the pixel starts at
    // parity s and ends at parity e, summed over the bits that are not modelled. Stored for the
    // backward pass. The symbols of a pixel are read against the running parity of its bits, which
    // makes the fit a product over bits; it is built level by level down the bit tree instead of
    // value by value.
    std::vector<float> weight(static_cast<std::size_t>(pixels) * 4 * plane);
    auto W = [&](int i, int s, int e) { return &weight[(static_cast<std::size_t>(i) * 4 + static_cast<std::size_t>(s * 2 + e)) * plane]; };

    constexpr float kClip = 14.0f;
    std::vector<float> fit0(plane), fit1(plane);               // start parity 0 and 1
    std::vector<float> tail0(static_cast<std::size_t>(X)), tail1(static_cast<std::size_t>(X));
    // buildTrees: out0[0..2^count) = product over bits of exp(+-l/2), the sign set by the running parity, for a start
    // parity of 0; out1 the same for a start parity of 1. Flipping the start parity swaps every factor for its
    // reciprocal, so both trees come from one exponential per bit.
    auto buildTrees = [&](const float* l, int count, float* out0, float* out1, const std::vector<uint8_t>& parityOf) {
        out0[0] = 1.0f; out1[0] = 1.0f;
        for (int k = 0; k < count; ++k) {
            const float c = std::max(-kClip, std::min(kClip, l[k]));
            const float up = std::exp(0.5f * c), down = 1.0f / up;   // parity 0 reads +llr, parity 1 reads -llr
            for (int j = (1 << k) - 1; j >= 0; --j) {
                const float b0 = out0[j], b1 = out1[j];
                const bool odd = parityOf[static_cast<std::size_t>(j)] != 0;
                const float fa = odd ? down : up, fb = odd ? up : down;
                out0[2 * j + 1] = b0 * fb; out0[2 * j] = b0 * fa;
                out1[2 * j + 1] = b1 * fa; out1[2 * j] = b1 * fb;
            }
        }
    };
    // vpow[k] = edge + (1 - edge) * a^|k - (S-1)|, k = 0 .. 2S-2: the vertical factor for any offset, read as a window
    std::vector<float> vpow;
    if (vertical.above) {
        const float a = static_cast<float>(std::exp(-1.0 / std::max(0.05, vertical.scale)));
        const float edge = static_cast<float>(vertical.edgeFraction);
        vpow.resize(static_cast<std::size_t>(2 * S - 1));
        for (int k = 0; k < 2 * S - 1; ++k) vpow[static_cast<std::size_t>(k)] = edge + (1.0f - edge) * std::pow(a, static_cast<float>(std::abs(k - (S - 1))));
    }
    std::vector<uint8_t> extraParity(static_cast<std::size_t>(X));
    for (int x = 0; x < X; ++x) extraParity[static_cast<std::size_t>(x)] = static_cast<uint8_t>(std::popcount(static_cast<unsigned>(x)) & 1);
    for (int i = 0; i < pixels; ++i) {
        const float* l = llr + static_cast<std::size_t>(i) * static_cast<std::size_t>(bits);
        buildTrees(l, chainBits, fit0.data(), fit1.data(), chainParity);
        // tailSum[r][par]: the summed-out bits start at parity r (what the chain left behind) and
        // the pixel ends at parity par xor r
        float tailSum[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        if (extra == 0) {
            tailSum[0][0] = 1.0f; tailSum[1][0] = 1.0f;
        } else {
            buildTrees(l + chainBits, extra, tail0.data(), tail1.data(), extraParity);
            for (int x = 0; x < X; ++x) {
                const int par = extraParity[static_cast<std::size_t>(x)];
                tailSum[0][par] += tail0[static_cast<std::size_t>(x)];
                tailSum[1][par] += tail1[static_cast<std::size_t>(x)];
            }
        }
        float top = 0.0f;
        for (int s = 0; s < 2; ++s)
            for (int e2 = 0; e2 < 2; ++e2) {
                float* w = W(i, s, e2);
                const float* fit = s ? fit1.data() : fit0.data();
                for (int v = 0; v < S; ++v) {
                    const int r = s ^ chainParity[static_cast<std::size_t>(v)];
                    const float val = fit[v] * tailSum[r][e2 ^ r];
                    w[v] = val;
                    top = std::max(top, val);
                }
            }
        if (vertical.above) {
            // pull towards the row above: edge + (1 - edge) * a^|v - u|. vfac <= 1, so the scale below stays safe.
            const int u = std::clamp(int(vertical.above[i]) >> extra, 0, S - 1);
            const float* pw = vpow.data() + (S - 1 - u);     // pw[v] = edge + (1-edge) * a^|v-u|
            for (int q = 0; q < 4; ++q) {
                float* w = W(i, q >> 1, q & 1);
                for (int v = 0; v < S; ++v) w[v] *= pw[v];
            }
        }
        if (top > 0.0f) {
            const float scale = 1.0f / top;
            for (int q = 0; q < 4; ++q) { float* w = W(i, q >> 1, q & 1); for (int v = 0; v < S; ++v) w[v] *= scale; }
        }
    }

    // Forward. F[i][e][v]: pixel i has value v and ends at parity e (i counts from 1; row 0 is the start).
    std::vector<float> forward(static_cast<std::size_t>(pixels + 1) * 2 * plane, 0.0f);
    auto F = [&](int i, int e) { return &forward[(static_cast<std::size_t>(i) * 2 + static_cast<std::size_t>(e)) * plane]; };
    for (int e = 0; e < 2; ++e) {
        const float v0 = startParity < 0 ? 0.5f / static_cast<float>(S) : (e == startParity ? 1.0f / static_cast<float>(S) : 0.0f);
        for (int v = 0; v < S; ++v) F(0, e)[v] = v0;
    }
    std::vector<float> mixed(plane), work(plane), mixed1(plane), work1(plane);
    for (int i = 1; i <= pixels; ++i) {
        float norm = 0.0f;
        for (int e = 0; e < 2; ++e) std::fill(F(i, e), F(i, e) + S, 0.0f);
        detail::applyPrior2(prior.scale, prior.edgeFraction, S, F(i - 1, 0), mixed.data(), work.data(),
                            F(i - 1, 1), mixed1.data(), work1.data());
        for (int s = 0; s < 2; ++s) {
            const float* mx = s ? mixed1.data() : mixed.data();
            for (int e = 0; e < 2; ++e) {
                const float* w = W(i - 1, s, e);
                float* dst = F(i, e);
                for (int v = 0; v < S; ++v) dst[v] += mx[v] * w[v];
            }
        }
        for (int e = 0; e < 2; ++e) for (int v = 0; v < S; ++v) norm += F(i, e)[v];
        const float inv = norm > 0.0f ? 1.0f / norm : 1.0f;
        for (int e = 0; e < 2; ++e) for (int v = 0; v < S; ++v) F(i, e)[v] *= inv;
    }

    // Backward, combined with the forward pass on the way down.
    std::vector<float> back(2 * plane, 1.0f), next(2 * plane), tmp(plane), tmp1(plane), post(plane, 0.0f);
    for (int i = pixels; i >= 1; --i) {
        // posterior of pixel i: F(i,e)[v] * back[e][v] summed over e (back = future given v and end parity)
        float total = 0.0f;
        for (int v = 0; v < S; ++v) {
            const float p = F(i, 0)[v] * back[static_cast<std::size_t>(v)]
                          + F(i, 1)[v] * back[plane + static_cast<std::size_t>(v)];
            post[static_cast<std::size_t>(v)] = p;
            total += p;
        }
        if (total > 0.0f) {
            double mean = 0.0; int arg = 0; float top = -1.0f;
            for (int v = 0; v < S; ++v) {
                const float p = post[static_cast<std::size_t>(v)] / total;
                mean += static_cast<double>(v) * p;
                if (p > top) { top = p; arg = v; }
            }
            out.mean[static_cast<std::size_t>(i - 1)] = static_cast<uint16_t>(std::lround(mean));
            out.best[static_cast<std::size_t>(i - 1)] = static_cast<uint16_t>(arg);
            out.confidence[static_cast<std::size_t>(i - 1)] = top;
        }
        // back for pixel i-1: sum over v' of T(v'|v) * (weight of pixel i given its start parity s = e_{i-1})
        for (int s = 0; s < 2; ++s) {
            float* t = s ? tmp1.data() : tmp.data();
            std::fill(t, t + plane, 0.0f);
            for (int e = 0; e < 2; ++e) {
                const float* w = W(i - 1, s, e);
                for (int v = 0; v < S; ++v) t[v] += w[v] * back[static_cast<std::size_t>(e) * plane + static_cast<std::size_t>(v)];
            }
        }
        detail::applyPrior2(prior.scale, prior.edgeFraction, S, tmp.data(), &next[0], work.data(),
                            tmp1.data(), &next[plane], work1.data());
        float norm = 0.0f;
        for (float x : next) norm += x;
        const float inv = norm > 0.0f ? 1.0f / norm : 1.0f;
        for (std::size_t k = 0; k < next.size(); ++k) back[k] = next[k] * inv;
    }
    return out;
}

}  // namespace nadirstation::softpixel
