#pragma once
// GOES-N/O/P GVAR link layer: soft-decision block deframer. No Qt, no I/O.
//
// What the downlink looks like (NOAA GOES-N/O/P OGE ICD, section 3, and the on-air stream itself):
//   * BPSK, 2.11136 Msym/s, differential NRZ-S (a transition is a logic 0, no transition a logic 1);
//   * the data is a stream of blocks: 10032 bit sync, 720 bit header (30 bytes sent three times),
//     the information field, a 16 bit CRC. Block lengths vary with the scan width;
//   * the sync code is the first 10032 bits of a 15 bit PN sequence (preset 051665 octal,
//     feedback from register bits 15 and 8). The same generator keeps running after the sync and
//     scrambles the rest of the block, and every other byte is complemented.
//
// Everything here works on soft values:
//   1. Block sync is found by correlating the received symbols against the NRZ-S encoded PN sync,
//      first on hard decisions over the last 512 sync symbols (cheap, popcount), then on soft
//      values over the whole 10032 symbols. The second step has a processing gain of 40 dB, so
//      the block boundary is found long after the symbol decisions have become useless.
//   2. NRZ-S is undone on soft values (max-log LLR of a difference), so a polarity flip of the
//      carrier loop costs one bit and no separate polarity search is needed.
//   3. The three copies of the header are summed as LLRs, then a CRC check with bit-flipping on
//      the least reliable bits recovers headers that hard decisions would lose.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <vector>

namespace nadirstation::gvar {

constexpr int kSyncBits = 10032;
constexpr int kHeaderBits = 720;
constexpr int kHeaderBytes = 30;
constexpr int kMaxBlockBits = 262288;   // sync + header + information + CRC at the widest scan
constexpr int kPnPeriod = 32767;
constexpr int kTemplateBits = 512;      // hard-decision pre-correlator length

struct Tables {
    std::array<uint8_t, kPnPeriod> pn{};            // PN bit i (period 32767)
    std::vector<float> syncSymbols;                 // NRZ-S encoded sync, +-1, kSyncBits long
    std::array<uint64_t, kTemplateBits / 64> tail{};// last 512 sync symbols as bits, newest in bit 0 of word 0
    std::array<uint16_t, 256> crc{};
    Tables() {
        uint16_t r = 0b101001110110101;
        for (int i = 0; i < kPnPeriod; ++i) {
            const int b = ((r >> 14) & 1) ^ ((r >> 7) & 1);
            r = uint16_t((r << 1) | b);
            pn[size_t(i)] = uint8_t(b);
        }
        syncSymbols.resize(kSyncBits);
        float t = 1.0f;
        for (int i = 0; i < kSyncBits; ++i) {
            if (!pn[size_t(i)]) t = -t;             // a transition for a logic 0
            syncSymbols[size_t(i)] = t;
        }
        for (int i = 0; i < kTemplateBits; ++i)
            if (syncSymbols[size_t(kSyncBits - 1 - i)] > 0) tail[size_t(i / 64)] |= uint64_t(1) << (i % 64);
        for (int i = 0; i < 256; ++i) {
            uint16_t c = uint16_t(i << 8);
            for (int k = 0; k < 8; ++k) c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
            crc[size_t(i)] = c;
        }
    }
    uint16_t crc16(const uint8_t* d, int n) const {   // CCITT, preset all ones, no final inversion
        uint16_t c = 0xFFFF;
        for (int i = 0; i < n; ++i) c = uint16_t((c << 8) ^ crc[size_t((c >> 8) ^ d[i])]);
        return c;
    }
};
inline const Tables& tables() { static const Tables t; return t; }

struct BlockHeader {
    int id = -1;            // 0 documentation, 1..10 imager, 11 sounder/other, 15 idle
    int wordSize = 0;       // bits per word: 6, 8 or 10
    int wordCount = 0;      // words in the information field, CRC included
    int productId = 0;
    int blockCount = 0;
    int bitsFixed = 0;      // bits the soft repair had to flip
    bool crcOk = false;
    bool plausible = false; // id and word size are legal and the always-zero fields are zero
    bool predicted = false; // CRC passed only after the fixed fields were taken from the previous header of the same block id
    bool guessed = false;   // CRC failed; id, word size and count were taken from the best-matching earlier header (setHeaderPrediction)
    uint8_t raw[kHeaderBytes] = {};
};

struct Block {
    BlockHeader header;
    std::vector<int8_t> llr;   // information field and CRC, +127 = certainly 1; the header is removed
    int64_t endSymbol = 0;     // stream position of the end of this block's sync
    float syncZ = 0.0f;        // correlation of the sync that opened the block, in sigma
    bool truncated = false;    // ended at the size limit rather than at the next sync
    // Soft symbols of the information field and CRC (same positions as llr), kept only when
    // Deframer::setKeepSymbols(true). Sign = BPSK decision, any scale; llrScale turns a symbol into
    // ln P(t=1)/P(t=0): 2 * amplitude / noise variance, measured on the sync word that opened the block.
    std::vector<float> sym;
    float llrScale = 0.0f;
};

struct Stats {
    uint64_t symbols = 0;
    uint64_t syncs = 0;          // sync words accepted
    uint64_t blocks = 0;         // blocks emitted
    uint64_t headerCrcOk = 0;
    uint64_t headerRepaired = 0; // of those, how many needed bit flips
    uint64_t headerPlausible = 0;
    uint64_t truncated = 0;
    uint64_t stage1 = 0, stage2 = 0;
    float lastZ = 0.0f;
    double berSum = 0.0;         // coded-symbol error rate over each accepted sync, summed
    uint64_t berCount = 0;
    uint64_t headerGuessed = 0;
    uint64_t headerRefused = 0;
    uint64_t headerSkipped = 0;  // sequence said which block it was, but the soft header disagreed  // CRC-repaired headers refused because they contradict the block sequence  // CRC failed, accepted by matching an earlier header of the same block id
    uint64_t falseSyncs = 0;     // sync-like patterns rejected because they fell inside a block of known length
};

class Deframer {
public:
    using Sink = std::function<void(Block&&)>;

    // Soft coded symbols, one per channel symbol, sign = BPSK decision. Any scale.
    void feed(const float* s, size_t n, const Sink& sink) {
        const Tables& T = tables();
        for (size_t i = 0; i < n; ++i) {
            const float v = s[i];
            ring_[size_t(count_ & kRingMask)] = v;
            // hard-decision history, newest symbol in bit 0 of word 0
            for (int w = kWords - 1; w > 0; --w) hist_[size_t(w)] = (hist_[size_t(w)] << 1) | (hist_[size_t(w - 1)] >> 63);
            hist_[0] = (hist_[0] << 1) | (v > 0.0f ? 1u : 0u);
            ++count_;
            ++stats_.symbols;
            energy_ += double(v) * v;
            if (inBlock_) {
                appendLlr(v);
                // The header is complete 720 symbols into the block: its word count says where the block
                // ends, so a "sync" found before that is a false one.
                if (cur_.size() == size_t(kHeaderBits)) {
                    earlyHeader_ = BlockHeader();
                    parseHeader(cur_.data(), earlyHeader_);
                    earlyValid_ = true;
                    // Only a header that needed (almost) no repair may veto syncs: with 1350 three-bit trials and the
                    // structural prediction, a noisy header passes the 16-bit CRC by chance often enough near 0 dB, and
                    // a false word count would then hide the next dozens of blocks.
                    const BlockHeader& e = earlyHeader_;
                    const size_t len = size_t(kHeaderBits) + size_t(std::max(0, e.wordSize)) * size_t(std::max(0, e.wordCount));
                    const bool trusted = e.crcOk && e.plausible && !e.predicted && !e.guessed && e.bitsFixed <= 1
                                      && (e.wordSize == 6 || e.wordSize == 8 || e.wordSize == 10) && e.wordCount > 0
                                      && len + size_t(kSyncBits) <= size_t(kMaxBlockBits);
                    expectBits_ = trusted ? len : 0;
                    // Below that the block sequence still knows: a header that clearly is the next one expected
                    // (id and block counter) has the length that id always has.
                    if (!expectBits_ && predOn_ && haveLast_) {
                        int comb[240];
                        combineHeader(cur_.data(), comb);
                        Cand c; double second = -9.0;
                        if (bestCandidate(comb, 0, c, second) && c.next && c.rho >= 0.45 && c.score - second >= predMargin_)
                            expectBits_ = size_t(kHeaderBits) + templateFor(c.id).info;
                    }
                }
            }
            prev_ = v;
            if (count_ < kTemplateBits) continue;
            int mismatch = 0;
            mismatch = hardMismatch(hist_.data(), T.tail.data());
            if (mismatch > stage1Limit_ && mismatch < kTemplateBits - stage1Limit_) continue;
            ++stats_.stage1;
            if (count_ - lastSync_ < 256) continue;
            // The PN scrambling keeps running through the data, so filler words (zeros, which block 0 has
            // thousands of) are sent as a stretch of the same PN sequence the sync is made of and correlate
            // with it. Inside a block whose length is known from its header, a sync cannot occur yet.
            if (inBlock_ && expectBits_ && cur_.size() + 192 < expectBits_ + size_t(kSyncBits)) { ++stats_.falseSyncs; continue; }
            // Fill words (zeros) are sent as the running PN sequence itself, so inside block 0 and 11 the sync pattern
            // comes round again every 32767 symbols after the real sync and correlates exactly as well. No GVAR block
            // is a whole number of PN periods long (75088, 218688, 219168 sync to sync), so a sync at a multiple of
            // the period from the last one is the PN repeating, not a block.
            if (lastSync_ > 0) {
                const int64_t d = count_ - lastSync_;
                const int64_t k = (d + kPnPeriod / 2) / kPnPeriod;
                if (k >= 1 && k <= 8 && std::llabs(d - k * int64_t(kPnPeriod)) <= 2) { ++stats_.falseSyncs; continue; }
            }
            float z = 0.0f;
            if (!correlate(z)) continue;
            ++stats_.stage2;
            acceptSync(z, sink);
        }
    }

    // Close the block in progress, e.g. when reception stops.
    void flush(const Sink& sink) {
        if (inBlock_) finishBlock(sink, false, true);
        inBlock_ = false;
    }
    void reset() { *this = Deframer(); }

    const Stats& stats() const { return stats_; }
    bool inBlock() const { return inBlock_; }
    // Hard-decision pre-correlator: blocks pass when at most this many of the last 512 symbols
    // disagree (or agree). 224 is 3 sigma for random data; a clean sync at 10 dB below the
    // symbol noise still passes (p = 0.33 gives 169 +- 11).
    // Hamming distance of the 512 newest hard symbols from the sync template. A plain build has no POPCNT
    // instruction and the compiler falls back to a library call per word; use the instruction where the CPU has it.
#if defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__)) && !defined(__POPCNT__)
    __attribute__((target("popcnt"))) static int mismatchHw(const uint64_t* a, const uint64_t* b) {
        int m = 0;
        for (int w = 0; w < kWords; ++w) m += int(__builtin_popcountll(a[w] ^ b[w]));
        return m;
    }
    static bool hasHwPopcnt() {
        static const bool v = [] { __builtin_cpu_init(); return __builtin_cpu_supports("popcnt") != 0; }();
        return v;
    }
#endif
    static int mismatchSwar(const uint64_t* a, const uint64_t* b) {
        int m = 0;
        for (int w = 0; w < kWords; ++w) {
            uint64_t x = a[w] ^ b[w];
            x = x - ((x >> 1) & 0x5555555555555555ull);
            x = (x & 0x3333333333333333ull) + ((x >> 2) & 0x3333333333333333ull);
            x = (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0Full;
            m += int((x * 0x0101010101010101ull) >> 56);
        }
        return m;
    }
    static int hardMismatch(const uint64_t* a, const uint64_t* b) {
#if defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__)) && !defined(__POPCNT__)
        if (hasHwPopcnt()) return mismatchHw(a, b);
        return mismatchSwar(a, b);
#else
        int m = 0;
        for (int w = 0; w < kWords; ++w) m += int(__builtin_popcountll(a[w] ^ b[w]));
        return m;
#endif
    }

    void setStage1Limit(int v) { stage1Limit_ = v; }
    // Soft correlation acceptance in sigma of the noise-only distribution.
    void setSyncSigma(float v) { syncSigma_ = v; }
    void setKeepSymbols(bool v) { keepSymbols_ = v; }
    // Header prediction (the FY-2 idea: a frame is not thrown away because its CRC fails). When the CRC of an
    // imager header cannot be repaired, the soft header is matched against the last good header of every block id
    // with the block counter advanced; the fixed fields (id, word size, word count, product, flags) decide, and the
    // received block length must fit. Costs nothing at high SNR (the CRC passes) and keeps whole scan lines near 0 dB.
    void setHeaderPrediction(bool on, float minRho = 0.22f, float margin = 0.06f) { predOn_ = on; predMin_ = minRho; predMargin_ = margin; }

private:
    static constexpr int kWords = kTemplateBits / 64;
    static constexpr size_t kRing = 16384, kRingMask = kRing - 1;

    bool correlate(float& z) {
        const Tables& T = tables();
        const int64_t have = std::min<int64_t>(count_, kSyncBits);
        if (have < 2048) return false;
        double sum = 0.0, energy = 0.0;
        for (int64_t i = 0; i < have; ++i) {
            const float v = ring_[size_t((count_ - 1 - i) & kRingMask)];
            sum += double(v) * T.syncSymbols[size_t(kSyncBits - 1 - i)];
            energy += double(v) * v;
        }
        const double rms = std::sqrt(energy / double(have));
        if (rms <= 0.0) return false;
        z = float(sum / (rms * std::sqrt(double(have))));
        stats_.lastZ = z;
        if (std::fabs(z) > syncSigma_ && have == kSyncBits) {
            int err = 0;
            const bool neg = sum < 0.0;
            syncNeg_ = neg;
            for (int64_t i = 0; i < have; ++i) {
                const float v = ring_[size_t((count_ - 1 - i) & kRingMask)];
                const bool rx = (neg ? -v : v) > 0.0f, tx = T.syncSymbols[size_t(kSyncBits - 1 - i)] > 0.0f;
                err += rx != tx;
            }
            stats_.berSum += double(err) / double(have);
            ++stats_.berCount;
            // Signal amplitude and noise variance from the known sync word (Gaussian noise about +-mean).
            const double mean = std::fabs(sum) / double(have);
            const double var = std::max(1e-6, energy / double(have) - mean * mean);
            pendingScale_ = float(2.0 * mean / var);
        }
        // The correlation is signed by the carrier polarity; either sign is a sync.
        return std::fabs(z) > syncSigma_;
    }

    void appendLlr(float v) {
        const Tables& T = tables();
        const size_t j = cur_.size();
        if (j >= size_t(kMaxBlockBits)) return;     // longer than any legal block: wait for the next sync
        // max-log LLR of a difference: positive when the two symbols agree (logic 1 in NRZ-S)
        const float a = v, b = prev_;
        float l = std::copysign(std::min(std::fabs(a), std::fabs(b)), a * b);
        // undo PN and the complement of odd-numbered (0 based) bytes
        const int flip = T.pn[(size_t(kSyncBits) + j) % kPnPeriod] ^ int((j >> 3) & 1);
        if (flip) l = -l;
        cur_.push_back(int8_t(std::clamp(int(std::lround(l * scale_)), -127, 127)));
        if (keepSymbols_ && j >= size_t(kHeaderBits)) sym_.push_back(v);
    }

    void acceptSync(float z, const Sink& sink) {
        lastSync_ = count_;
        ++stats_.syncs;
        if (inBlock_) finishBlock(sink, true, false);
        cur_.clear();
        sym_.clear();
        cur_.reserve(size_t(kMaxBlockBits));
        earlyValid_ = false;
        expectBits_ = 0;
        syncZ_ = z;
        syncScale_ = pendingScale_;
        // the carrier polarity of the sync word is the polarity of this block's symbols
        polarity_ = syncNeg_ ? -1.0f : 1.0f;
        inBlock_ = true;
        // Level for the LLR quantiser: about 1/4 of the mean symbol amplitude per count of 32.
        const double rms = std::sqrt(energy_ / double(std::max<uint64_t>(stats_.symbols, 1)));
        scale_ = rms > 0.0 ? float(48.0 / rms) : 48.0f;
    }

    void finishBlock(const Sink& sink, bool atSync, bool flushed) {
        std::vector<int8_t> data = std::move(cur_);
        cur_.clear();
        if (atSync) {
            if (data.size() < size_t(kSyncBits)) return;
            data.resize(data.size() - size_t(kSyncBits));       // the next block's sync is in there
        }
        if (data.size() < size_t(kHeaderBits) + 16) return;
        Block b;
        b.syncZ = syncZ_;
        b.endSymbol = count_;
        b.truncated = flushed || data.size() >= size_t(kMaxBlockBits - kSyncBits);
        b.llrScale = syncScale_;
        if (keepSymbols_) {
            // sym_ ran to the end of the next block's sync when it was cut at a sync: trim to the data
            const size_t want = data.size() - size_t(kHeaderBits);
            if (sym_.size() > want) sym_.resize(want);
            for (auto& x : sym_) x *= polarity_;
            b.sym = std::move(sym_);
            sym_.clear();
        }
        if (earlyValid_) b.header = earlyHeader_;
        else parseHeader(data.data(), b.header);
        if (predOn_) trackHeader(data.data(), data.size() - size_t(kHeaderBits), b.truncated, b.header);
        earlyValid_ = false;
        expectBits_ = 0;
        data.erase(data.begin(), data.begin() + kHeaderBits);
        b.llr = std::move(data);
        ++stats_.blocks;
        if (b.header.crcOk && !b.header.guessed) { ++stats_.headerCrcOk; if (b.header.bitsFixed) ++stats_.headerRepaired; }
        if (b.header.plausible) ++stats_.headerPlausible;
        if (b.truncated) ++stats_.truncated;
        sink(std::move(b));
    }

    // Sum the three copies, vote, check the CRC, and if it fails flip the least reliable bits.
    void parseHeader(const int8_t* l, BlockHeader& h) {
        const Tables& T = tables();
        int comb[240];
        for (int i = 0; i < 240; ++i) comb[i] = int(l[i]) + int(l[240 + i]) + int(l[480 + i]);
        // Bits known to be zero in every header: the high nibble of word 1 and the spare long word.
        const auto pin = [&](int bit) { comb[bit] = -3 * 127; };
        // Byte 0 is the block id: 0..11 or 15 with the high nibble clear, or 240 (0xF0) for block 0, so
        // the four high bits are all equal and are decided together. The spare long word is zero.
        {
            const int nib = comb[0] + comb[1] + comb[2] + comb[3];
            for (int b = 0; b < 4; ++b) comb[b] = nib;
        }
        for (int b = 24 * 8; b < 28 * 8; ++b) pin(b);
        uint8_t raw[kHeaderBytes] = {};
        for (int i = 0; i < 240; ++i) if (comb[i] > 0) raw[i / 8] |= uint8_t(0x80 >> (i % 8));
        const auto crcGood = [&](const uint8_t* r) {
            const uint16_t calc = T.crc16(r, 28);
            const uint16_t stored = uint16_t((r[28] << 8) | r[29]);
            return calc == uint16_t(~stored);
        };
        std::memcpy(h.raw, raw, sizeof raw);
        if (crcGood(raw)) {
            h.crcOk = true;
        } else {
            // 20 weakest bits (not the pinned ones), try every subset of up to 3
            std::array<int, 20> weak{};
            std::array<int, 240> order{};
            for (int i = 0; i < 240; ++i) order[size_t(i)] = i;
            std::partial_sort(order.begin(), order.begin() + 20, order.end(),
                              [&](int a, int b) { return std::abs(comb[a]) < std::abs(comb[b]); });
            for (int i = 0; i < 20; ++i) weak[size_t(i)] = order[size_t(i)];
            int bestCost = 1 << 30;
            uint8_t best[kHeaderBytes];
            int bestFlips = 0;
            const auto trial = [&](int n, const int* idx) {
                uint8_t t[kHeaderBytes];
                std::memcpy(t, raw, sizeof t);
                int cost = 0;
                for (int k = 0; k < n; ++k) { t[idx[k] / 8] ^= uint8_t(0x80 >> (idx[k] % 8)); cost += std::abs(comb[idx[k]]); }
                if (cost < bestCost && crcGood(t)) { bestCost = cost; std::memcpy(best, t, sizeof t); bestFlips = n; }
            };
            for (int a = 0; a < 20; ++a) {
                int i1[1] = {weak[size_t(a)]}; trial(1, i1);
                for (int b = a + 1; b < 20; ++b) {
                    int i2[2] = {weak[size_t(a)], weak[size_t(b)]}; trial(2, i2);
                    for (int c = b + 1; c < 20; ++c) { int i3[3] = {weak[size_t(a)], weak[size_t(b)], weak[size_t(c)]}; trial(3, i3); }
                }
            }
            if (bestCost < (1 << 30)) { std::memcpy(h.raw, best, sizeof best); h.crcOk = true; h.bitsFixed = bestFlips; }
            else if (predictHeader(comb, h)) { h.crcOk = true; h.predicted = true; }
        }
        if (h.crcOk) rememberHeader(h);
        fillFields(h);
    }

    static void fillFields(BlockHeader& h) {
        const uint8_t* r = h.raw;
        h.id = r[0] & 0x0F;
        h.wordSize = r[1];
        h.wordCount = (r[2] << 8) | r[3];
        h.productId = (r[4] << 8) | r[5];
        h.blockCount = ((r[12] << 8) | r[13]) & 0x7FF;
        const bool idOk = (h.id <= 11) || h.id == 15 || r[0] == 240;
        if (r[0] == 240) h.id = 0;       // block 0 is sent as 240
        const bool sizeOk = h.wordSize == 6 || h.wordSize == 8 || h.wordSize == 10;
        h.plausible = h.crcOk || (idOk && sizeOk && !r[24] && !r[25] && !r[26] && !r[27]);
    }

    // ---- block sequence tracker ------------------------------------------------------------------
    // What the data format guarantees (seen on GOES-13..15): the block counter (bytes 12-13) goes up by one with
    // every block, ids run 0, 1..10, then 0 or 11 (11 can repeat), and each id always has the same length (block
    // 0 and 11: 64336 information bits, 1-2: 208416, 3-10: 207936) and the same bytes 0..11 and 14..15. The
    // block length is measured independently by the distance between syncs, which the 10032-symbol PN finds
    // reliably far below the point where headers fail. So a header is predicted completely from the previous
    // one and checked against the soft bits; a CRC repair that contradicts the sequence is refused.
    struct Track { bool have = false; uint8_t raw[kHeaderBytes] = {}; size_t info = 0; };
    struct Cand { int id = -1; uint16_t count = 0; double rho = -2.0, score = -9.0; bool next = false; };

    // The imager blocks of one kind differ only in the id byte (and the counter): 1-2 carry the IR channels, 3-10
    // the eight visible detectors. A template for an id not seen yet is made from a sibling.
    Track derived_;
    const Track& templateFor(int id) const {
        const Track& t = trk_[size_t(id & 15)];
        if (t.have) return t;
        const int lo = (id == 1 || id == 2) ? 1 : (id >= 3 && id <= 10) ? 3 : -1;
        const int hi = lo == 1 ? 2 : lo == 3 ? 10 : -1;
        if (lo < 0) return t;
        for (int k = lo; k <= hi; ++k) {
            if (!trk_[size_t(k)].have) continue;
            Track& d = const_cast<Track&>(derived_);
            d = trk_[size_t(k)];
            d.raw[0] = uint8_t(id);
            return derived_;
        }
        return t;
    }
    // Soft sum of the header bits that never change within one kind of block (bytes 1..11 and 14..15), per block
    // length. Three copies a block, so after a line of blocks they are clean even at 20 % symbol errors.
    struct Acc { size_t info = 0; int n = 0; int64_t sum[16 * 8] = {}; };
    std::array<Acc, 10> acc_{};
    int pos_ = 0;
    bool countKnown_ = false, posKnown_ = false;
    // The short blocks between imager lines: block 0 is always 8040 documentation bytes plus the CRC (64336 bits in 8-bit
    // words); block 11 has the length its header gave the last time one was read. Imager blocks are not recognised by
    // length: in sector scans they are as short as these (8850 words).
    bool isShort(size_t infoBits) const {
        const auto near = [](size_t a, size_t b) { return a + 16 >= b && a <= b + 16; };
        if (near(infoBits, 64336)) return true;
        for (int id : {0, 11}) if (trk_[size_t(id)].have && near(infoBits, trk_[size_t(id)].info)) return true;
        // otherwise from the header itself: blocks of this length, soft-summed, say 8-bit words (documentation)
        return accWordSize(infoBits) == 8;
    }
    // Word size (header byte 1) of the blocks of this length, from the soft sum of their headers; 0 = not known yet.
    int accWordSize(size_t infoBits) const {
        for (const auto& x : acc_) {
            if (!x.n || infoBits + 16 < x.info || infoBits > x.info + 16) continue;
            if (x.n < 3) return 0;
            int ws = 0;
            for (int i = 8; i < 16; ++i) ws = (ws << 1) | (x.sum[i] > 0 ? 1 : 0);
            return ws;
        }
        return 0;
    }
    void accumulate(const int* comb, size_t infoBits) {
        if (infoBits < 4000) return;
        Acc* a = nullptr;
        for (auto& x : acc_) if (x.n && infoBits + 16 >= x.info && infoBits <= x.info + 16) { a = &x; break; }
        if (!a) {
            a = &acc_[0];
            for (auto& x : acc_) if (x.n < a->n) a = &x;
            *a = Acc(); a->info = infoBits;
        }
        if (a->n >= 4096) return;
        ++a->n;
        for (int i = 8; i < 16 * 8; ++i) if (i < 12 * 8 || i >= 14 * 8) a->sum[i] += comb[i];
    }
    // Header for imager block `id` with `infoBits` of information: the learnt header of that id or a sibling of
    // the same length, otherwise the accumulated bits, accepted only when word size and count fit the length.
    bool templateBytes(int id, size_t infoBits, uint8_t* raw) const {
        const Track& t = templateFor(id);
        if (t.have && infoBits + 16 >= t.info && infoBits <= t.info + 16) { std::memcpy(raw, t.raw, kHeaderBytes); raw[0] = uint8_t(id); return true; }
        for (const auto& x : acc_) {
            if (!x.n || infoBits + 16 < x.info || infoBits > x.info + 16 || x.n < 4) continue;
            std::memset(raw, 0, kHeaderBytes);
            raw[0] = uint8_t(id);
            for (int i = 8; i < 16 * 8; ++i) if ((i < 12 * 8 || i >= 14 * 8) && x.sum[i] > 0) raw[i / 8] |= uint8_t(0x80 >> (i % 8));
            for (const auto& k : trk_) if (k.have) { std::memcpy(raw + 16, k.raw + 16, 8); break; }
            const int ws = raw[1], wc = (raw[2] << 8) | raw[3];
            const long fit = long(ws) * wc - long(infoBits);
            if (ws != 10 || fit < -16 || fit > 16) return false;
            return true;
        }
        return false;
    }
    static double headerRho(const int* comb, const uint8_t* raw, uint16_t count) {
        uint8_t e[16];
        std::memcpy(e, raw, 16);
        e[12] = uint8_t(count >> 8); e[13] = uint8_t(count & 0xFF);
        double sum = 0.0, norm = 0.0;
        for (int i = 0; i < 16 * 8; ++i) { sum += ((e[i / 8] >> (7 - i % 8)) & 1) ? comb[i] : -comb[i]; norm += std::abs(double(comb[i])); }
        return norm > 0.0 ? sum / norm : -1.0;
    }
    static void combineHeader(const int8_t* l, int* comb) {
        for (int i = 0; i < 240; ++i) comb[i] = int(l[i]) + int(l[240 + i]) + int(l[480 + i]);
        const int nib = comb[0] + comb[1] + comb[2] + comb[3];
        for (int b = 0; b < 4; ++b) comb[b] = nib;
    }
    bool expectedNext(int id) const {
        // 0 and 11 have the same length and nearly the same header; treat them as one step of the sequence
        if (lastId_ == 0 || lastId_ == 11) return id == 1 || id == 0 || id == 11;
        if (lastId_ >= 1 && lastId_ <= 9) return id == lastId_ + 1;
        if (lastId_ == 10) return id == 0 || id == 11;
        return false;
    }
    // Best (id, counter) for the soft header among the ids seen so far, counter = last + 1..4. infoBits = 0: length
    // not known yet (early, from the first 720 symbols).
    bool bestCandidate(const int* comb, size_t infoBits, Cand& best, double& second) const {
        best = Cand(); second = -9.0;
        double norm = 0.0;
        for (int i = 0; i < 16 * 8; ++i) norm += std::abs(double(comb[i]));
        if (norm <= 0.0) return false;
        for (int id = 0; id < 16; ++id) {
            const Track& t = templateFor(id);
            if (!t.have) continue;
            if (infoBits && (infoBits + 16 < t.info || infoBits > t.info + 16)) continue;
            const bool nx = expectedNext(id);
            double idBest = -9.0, idRho = -2.0; int idDelta = 1;
            for (int delta = 1; delta <= 4; ++delta) {
                uint8_t e[16];
                std::memcpy(e, t.raw, 16);
                const uint16_t c = uint16_t(lastCount_ + delta);
                e[12] = uint8_t(c >> 8); e[13] = uint8_t(c & 0xFF);
                double sum = 0.0;
                for (int i = 0; i < 16 * 8; ++i) sum += ((e[i / 8] >> (7 - i % 8)) & 1) ? comb[i] : -comb[i];
                const double rho = sum / norm;
                const double score = rho + (nx ? (delta == 1 ? 0.10 : 0.03) : 0.0) - 0.01 * (delta - 1);
                if (score > idBest) { idBest = score; idRho = rho; idDelta = delta; }
            }
            if (idBest > best.score) {
                second = best.score;
                best.score = idBest; best.rho = idRho; best.id = id; best.count = uint16_t(lastCount_ + idDelta); best.next = nx && idDelta == 1;
            } else if (idBest > second) second = idBest;
        }
        return best.id >= 0;
    }

    void trackHeader(const int8_t* l, size_t infoBits, bool truncated, BlockHeader& h) {
        int comb[240];
        combineHeader(l, comb);
        if (!truncated) accumulate(comb, infoBits);
        // Without any header yet, start the sequence from the block lengths: a short block followed by a long one.
        if (!haveLast_ && !truncated && !h.crcOk) {
            if (isShort(infoBits)) { haveLast_ = true; lastId_ = 0; pos_ = 0; posKnown_ = true; }
            return;
        }
        Cand c; double second = -9.0;
        const bool have = haveLast_ && !truncated && bestCandidate(comb, infoBits, c, second);
        const bool candGood = have && c.rho >= predMin_ && c.score - second >= predMargin_;
        if (h.crcOk && haveLast_) {
            const int cnt = (h.raw[12] << 8) | h.raw[13];
            const int id = h.raw[0] == 240 ? 0 : (h.raw[0] & 15);
            const int step = (cnt - int(lastCount_)) & 0xFFFF;
            const Track& t = templateFor(id);
            const bool lenOk = truncated || !t.have || (infoBits + 16 >= t.info && infoBits <= t.info + 16);
            const bool exact = h.bitsFixed == 0 && !h.predicted;
            const bool idOk = (id == 0 || id == 11) ? isShort(infoBits) || truncated : (id == pos_ + 1 || !posKnown_);
            bool fieldsOk = true;                     // the fixed fields must be those this kind of block always has
            if (id >= 1 && id <= 10) {
                uint8_t ref[kHeaderBytes];
                if (templateBytes(id, infoBits, ref)) fieldsOk = std::memcmp(ref + 1, h.raw + 1, 11) == 0;
            }
            const bool consistent = lenOk && fieldsOk && idOk && (!countKnown_ || step == 1 || (step >= 2 && step <= 8 && !posKnown_));
            if (!consistent && !exact) {
                ++stats_.headerRefused;
                h.crcOk = false; h.predicted = false; h.bitsFixed = 0;
                fillFields(h);
            }
        }
        if (!h.crcOk && candGood) {
            const Track& t = templateFor(c.id);
            std::memcpy(h.raw, t.raw, kHeaderBytes);
            h.raw[12] = uint8_t(c.count >> 8); h.raw[13] = uint8_t(c.count & 0xFF);
            h.crcOk = true; h.guessed = true; h.bitsFixed = -1;
            fillFields(h);
            ++stats_.headerGuessed;
            lastId_ = c.id; lastCount_ = c.count;
            pos_ = (c.id >= 1 && c.id <= 10) ? c.id : 0;
            // Block 0 and 11 carry documentation and telemetry that need every bit: they move the sequence on
            // but are not handed on as good.
            if (c.id == 0 || c.id == 11) { h.crcOk = false; h.plausible = false; }
            return;
        }
        if (!h.crcOk) {
            // Flywheel: syncs are found far below the point where headers fail, so the block sequence is known from
            // the sync distances alone. 0 and 11 are the short blocks; every imager line is preceded by one of them
            // and its blocks then run 1..10. The id is the position since the last short block.
            if (!haveLast_ || truncated) return;
            const bool shortBlk = isShort(infoBits);
            const int id = shortBlk ? 0 : pos_ + 1;
            lastCount_ = uint16_t(lastCount_ + 1);
            lastId_ = shortBlk ? 0 : id;
            pos_ = shortBlk ? 0 : id;
            if (id > 10) posKnown_ = false;           // lost track: wait for the next documentation block
            if (shortBlk || id > 10) return;          // documentation/telemetry needs every bit
            uint8_t raw[kHeaderBytes];
            if (!templateBytes(id, infoBits, raw)) { ++stats_.headerSkipped; return; }
            raw[12] = uint8_t(lastCount_ >> 8); raw[13] = uint8_t(lastCount_ & 0xFF);
            const double r = headerRho(comb, raw, lastCount_);
            if (r < seqMin_) { ++stats_.headerSkipped; return; }
            std::memcpy(h.raw, raw, kHeaderBytes);
            h.crcOk = true; h.guessed = true; h.bitsFixed = -2;
            fillFields(h);
            ++stats_.headerGuessed;
            return;
        }
        const int id = h.raw[0] == 240 ? 0 : (h.raw[0] & 15);
        lastId_ = id; lastCount_ = uint16_t((h.raw[12] << 8) | h.raw[13]); haveLast_ = true;
        pos_ = (id >= 1 && id <= 10) ? id : 0;
        countKnown_ = true; posKnown_ = true;
        if (!truncated && !h.predicted && h.bitsFixed <= 3) {
            Track& t = trk_[size_t(id)];
            t.have = true; std::memcpy(t.raw, h.raw, kHeaderBytes); t.info = infoBits;
        }
    }

    // ---- header recovery from structure ----------------------------------------------------------
    // GVAR has no forward error correction; at a symbol error rate near 1% a header can lose more bits
    // than the 3-bit CRC search repairs. But most of a header does not change from one block to the next
    // block of the same id: word size, word count, product id, flags, SPS id and range word are
    // identical, the two spare words are zero. Taking those bytes from the last good header of that id
    // (when the received bits agree with them to within a few weak bits) leaves only the block counter
    // and the time code unknown; those are searched under the CRC plus strict plausibility (BCD digits,
    // time within seconds of the last header, counter just ahead of the last one).
    struct Pred {
        bool have = false;
        uint8_t raw[kHeaderBytes] = {};
        int count = 0;
        int64_t sod = -1;
    };
    Pred pred_[16];
    BlockHeader earlyHeader_;
    bool earlyValid_ = false;
    size_t expectBits_ = 0;
    uint64_t predictedHeaders_ = 0;

    static int64_t bcdStamp(const uint8_t* t) {      // 8 BCD bytes YYYYDDDHHMMSSmmm -> day * 86400 + second of day, ms ignored
        int d[16];
        for (int i = 0; i < 8; ++i) { d[2 * i] = t[i] >> 4; d[2 * i + 1] = t[i] & 15; }
        for (int i = 0; i < 16; ++i) if (d[i] > 9) return -1;
        const int year = d[0] * 1000 + d[1] * 100 + d[2] * 10 + d[3];
        const int doy = d[4] * 100 + d[5] * 10 + d[6];
        const int hour = d[7] * 10 + d[8], minute = d[9] * 10 + d[10], sec = d[11] * 10 + d[12];
        if (year < 1990 || year > 2100 || doy < 1 || doy > 366 || hour > 23 || minute > 59 || sec > 59) return -1;
        return int64_t(year) * 400 * 86400 + int64_t(doy) * 86400 + hour * 3600 + minute * 60 + sec;
    }

    void rememberHeader(const BlockHeader& h) {
        const int id = h.raw[0] == 240 ? 0 : (h.raw[0] & 15);   // h.id is not filled in yet when this runs
        Pred& p = pred_[id];
        p.have = true;
        std::memcpy(p.raw, h.raw, kHeaderBytes);
        p.count = (h.raw[12] << 8) | h.raw[13];
        p.sod = bcdStamp(h.raw + 16);
    }

    bool predictHeader(const int* comb, BlockHeader& h) {
        // 1. which id's last header do the received bits resemble? Compare the fixed bytes.
        static const int kFixed[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15, 24, 25, 26, 27};
        int bestId = -1, bestBad = 1 << 30;
        for (int id = 0; id < 16; ++id) {
            if (!pred_[id].have) continue;
            int bad = 0;
            for (int by : kFixed)
                for (int b = 0; b < 8; ++b) {
                    const int bit = by * 8 + b;
                    const bool want = (pred_[id].raw[by] >> (7 - b)) & 1;
                    if ((comb[bit] > 0) != want) ++bad;
                }
            if (bad < bestBad) { bestBad = bad; bestId = id; }
        }
        if (bestId < 0 || bestBad > 6) return false;
        const Pred& P = pred_[bestId];
        // 2. pin the fixed bytes (and the id byte) to the predicted values
        int c[240];
        std::memcpy(c, comb, sizeof c);
        const auto pinByte = [&](int by, uint8_t v) {
            for (int b = 0; b < 8; ++b) c[by * 8 + b] = ((v >> (7 - b)) & 1) ? 3 * 127 : -3 * 127;
        };
        pinByte(0, P.raw[0]);
        for (int by : kFixed) pinByte(by, P.raw[by]);
        // 3. free bits: block counter (bytes 12-13) and the time code (16-23)
        std::vector<int> freeBits;
        for (int b = 12 * 8; b < 14 * 8; ++b) freeBits.push_back(b);
        for (int b = 16 * 8; b < 24 * 8; ++b) freeBits.push_back(b);
        std::sort(freeBits.begin(), freeBits.end(), [&](int a, int b) { return std::abs(c[a]) < std::abs(c[b]); });
        constexpr int kWeak = 22;
        freeBits.resize(kWeak);
        uint8_t base[kHeaderBytes] = {};
        for (int i = 0; i < 240; ++i) if (c[i] > 0) base[i / 8] |= uint8_t(0x80 >> (i % 8));
        const Tables& T = tables();
        const auto acceptable = [&](const uint8_t* r) {
            const uint16_t calc = T.crc16(r, 28);
            if (calc != uint16_t(~uint16_t((r[28] << 8) | r[29]))) return false;
            const int cnt = (r[12] << 8) | r[13];
            if (((cnt - P.count) & 0xFFFF) > 64) return false;
            const int64_t t = bcdStamp(r + 16);
            if (t < 0) return false;
            if (P.sod >= 0 && (t < P.sod - 2 || t > P.sod + 40)) return false;
            return true;
        };
        int bestCost = 1 << 30;
        uint8_t best[kHeaderBytes];
        int bestFlips = 0, accepted = 0;
        const auto trial = [&](int n, const int* idx) {
            uint8_t t[kHeaderBytes];
            std::memcpy(t, base, sizeof t);
            int cost = 0;
            for (int k = 0; k < n; ++k) { t[idx[k] / 8] ^= uint8_t(0x80 >> (idx[k] % 8)); cost += std::abs(c[idx[k]]); }
            if (!acceptable(t)) return;
            ++accepted;
            if (cost < bestCost) { bestCost = cost; std::memcpy(best, t, sizeof t); bestFlips = n; }
        };
        { int none[1] = {0}; trial(0, none); }
        for (int a = 0; a < kWeak; ++a) {
            int i1[1] = {freeBits[size_t(a)]}; trial(1, i1);
            for (int b = a + 1; b < kWeak; ++b) {
                int i2[2] = {freeBits[size_t(a)], freeBits[size_t(b)]}; trial(2, i2);
                for (int d = b + 1; d < kWeak; ++d) {
                    int i3[3] = {freeBits[size_t(a)], freeBits[size_t(b)], freeBits[size_t(d)]}; trial(3, i3);
                    for (int e = d + 1; e < kWeak; ++e) {
                        int i4[4] = {freeBits[size_t(a)], freeBits[size_t(b)], freeBits[size_t(d)], freeBits[size_t(e)]};
                        trial(4, i4);
                    }
                }
            }
        }
        // More than one acceptable candidate that differ is ambiguous: take the cheapest only when it
        // is unique, otherwise give up rather than guess.
        if (bestCost == (1 << 30) || accepted != 1) return false;
        std::memcpy(h.raw, best, sizeof best);
        h.bitsFixed = bestFlips + 100;       // 100+: predicted header
        ++predictedHeaders_;
        return true;
    }

    std::array<uint64_t, kWords> hist_{};
    std::array<float, kRing> ring_{};
    int64_t count_ = 0, lastSync_ = -1000000;
    double energy_ = 0.0;
    float prev_ = 0.0f, scale_ = 48.0f, syncZ_ = 0.0f;
    bool inBlock_ = false;
    std::vector<int8_t> cur_;
    std::vector<float> sym_;
    bool keepSymbols_ = false;
    float pendingScale_ = 0.0f, syncScale_ = 0.0f, polarity_ = 1.0f;
    bool syncNeg_ = false;
    int stage1Limit_ = 224;
    float syncSigma_ = 9.0f;
    bool predOn_ = false;
    float predMin_ = 0.22f, predMargin_ = 0.06f, seqMin_ = 0.03f;
    int lastId_ = -1;
    uint16_t lastCount_ = 0;
    bool haveLast_ = false;
    std::array<Track, 16> trk_{};
    Stats stats_;
};

}  // namespace nadirstation::gvar
