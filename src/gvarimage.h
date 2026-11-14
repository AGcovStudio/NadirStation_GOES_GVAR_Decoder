#pragma once
// GOES GVAR imager assembler: turns deframed blocks (gvardecoder.h) into the five channel rasters.
// No Qt, no I/O. Memory: rows are allocated only when a line arrives (a full-disk visible raster is
// 20944 x 10832 x 2 bytes = 454 MB, but a sector scan or a partial reception takes what it received).
//
//   * lines are filed by the relative scan count of the line documentation (first 16 words of every
//     imager block), taken as the majority over the blocks of one 10-block series, so a corrupted
//     counter in one block costs nothing; block ids come from the header, which is already a vote of
//     three copies and a CRC repair;
//   * pixel words are 10 bits; with soft recovery on, the 6 leading bits of every pixel are re-estimated
//     from the soft symbols and the neighbouring pixels (the FY-2 method, softpixel.h), the 4 trailing
//     bits keep their hard decisions.
#include "gvardecoder.h"
#include "softpixel.h"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <ctime>
#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <vector>

namespace nadirstation::gvar {

constexpr int kVisWidth = 20944, kIrWidth = 5236, kMaxCounter = 1354;
constexpr int kVisRows = kMaxCounter * 8, kIrRows = kMaxCounter * 2;

struct Raster16 {
    int width = 0;
    std::vector<std::unique_ptr<uint16_t[]>> rows;
    void init(int w, int h) { width = w; rows.clear(); rows.resize(size_t(h)); }
    int height() const { return int(rows.size()); }
    uint16_t* row(int y) {
        if (y < 0 || y >= height()) return nullptr;
        if (!rows[size_t(y)]) rows[size_t(y)].reset(new uint16_t[size_t(width)]());
        return rows[size_t(y)].get();
    }
    const uint16_t* peek(int y) const { return (y < 0 || y >= height()) ? nullptr : rows[size_t(y)].get(); }
    int firstRow() const { for (int y = 0; y < height(); ++y) if (rows[size_t(y)]) return y; return -1; }
    int lastRow() const { for (int y = height() - 1; y >= 0; --y) if (rows[size_t(y)]) return y; return -1; }
    void clear() { rows.clear(); width = 0; }
};

struct Block0Info {
    bool valid = false;
    int iwfpx = 0, iefpx = 0, infln = 0, isfln = 0, insln = 0, risct = 0;
    double subLon = 0.0, subLat = 0.0;
    std::time_t frameStart = 0, current = 0;
};

struct Observation {
    Raster16 ch[5];                 // 0 = visible (20944 wide), 1..4 = IR 3.9 / 6.5 / 10.7 / 13.3 (5236 wide)
    Block0Info info;
    int spacecraft = 0;
    int lines = 0, firstCounter = 0, lastCounter = 0;
    uint64_t blocksSeen = 0, blocksCrcOk = 0, blocksRecovered = 0;
    int visWidthWords = 0, irRecordWords = 0;   // widths actually transmitted
    bool softUsed = false;
    // Blocks whose header passed the CRC but whose information field ended early. Their rows are left out of the
    // picture (the image fills them from the neighbouring lines) rather than written as a short stripe of pixels
    // followed by black. truncatedLen* keep the first few word counts seen, to show where the cut happens.
    uint64_t blocksTruncated = 0, headersCorrected = 0;
    int truncatedById[11] = {};
    int truncatedWords[8] = {};
    int truncatedWordsN = 0;
    // IR rows whose record length came out shorter than the one every other line has (the line's own document
    // said so, or the information field did): the length of the normal lines is used instead. deadTailRows counts
    // rows whose last 256 words were all zero even so; they are left out and the picture fills them from their
    // neighbours. diag keeps the first few events: line counter, block id, record words used, record words found,
    // block words, declared words, document word count.
    uint64_t recOverridden = 0, deadTailRows = 0;
    int diag[8][7] = {};
    int diagN = 0;
};

// ---- helpers shared with the tests --------------------------------------------------------------

// Cumulative parity of (descrambling mask xor 1) from the first bit of the information field.
inline const std::vector<uint8_t>& runningMask() {
    static const std::vector<uint8_t> table = [] {
        const Tables& T = tables();
        std::vector<uint8_t> m(static_cast<size_t>(kMaxBlockBits), uint8_t(0));
        uint8_t acc = 0;
        for (size_t n = 0; n < m.size(); ++n) {
            const size_t j = size_t(kHeaderBits) + n;      // position inside the block after the sync
            const int mask = T.pn[(size_t(kSyncBits) + j) % kPnPeriod] ^ int((j >> 3) & 1);
            acc ^= uint8_t(mask ^ 1);
            m[n] = acc;
        }
        return m;
    }();
    return table;
}

struct LineDoc {
    int scId = 0, detector = 0, channel = 0, relScan = 0, pixelCount = 0, wordCount = 0;
};
inline LineDoc parseDoc(const uint16_t* w) {
    LineDoc d;
    d.scId = w[0]; d.detector = w[3]; d.channel = w[4];
    d.relScan = ((w[5] << 10) | w[6]) & 0x7FF;
    d.pixelCount = (w[9] << 10) | w[10];
    d.wordCount = ((w[11] << 10) | w[12]) & 0x1FFF;
    return d;
}

struct SoftParams {
    bool enabled = true;
    int chainBits = 6;            // leading bits of each 10-bit pixel that are re-estimated
    int chunk = 384;              // pixels per forward-backward pass
    int overlap = 24;             // extra pixels each side, discarded
    softpixel::Prior prior{1.0, 0.03};
    int threads = 0;              // 0 = hardware concurrency - 1 (at least 1)
    bool vertical = true;         // use the decoded row above (neighbouring detector of the channel) as a prior
    double vScale = 1.5, vEdge = 0.15;
    std::function<void()> threadInit;   // run at the start of each worker thread (e.g. to lower its priority)
};

// Re-estimate the leading chainBits of `count` consecutive 10-bit words starting at word `first` of the
// information field. `words` is updated in place. sym: soft symbols of the field, scale: LLR scale.
// above: optional 10-bit values of the row above (the neighbouring detector), one per word of [first, first + count).
inline void recoverWords(const float* sym, size_t nsym, float scale, uint16_t* words, int first, int count,
                         const SoftParams& sp, const uint16_t* above = nullptr) {
    if (count <= 0 || scale <= 0.0f) return;
    const auto& M = runningMask();
    const int cb = std::clamp(sp.chainBits, 1, 10);
    const int kHalf = sp.overlap;
    struct Chunk { int begin, end; };       // words [begin, end) are decoded, [keepBegin, keepEnd) kept
    std::vector<Chunk> chunks;
    for (int s = 0; s < count; s += sp.chunk) chunks.push_back({s, std::min(count, s + sp.chunk)});
    std::atomic<size_t> next{0};
    auto worker = [&] {
        std::vector<float> llr;
        for (;;) {
            const size_t ci = next.fetch_add(1);
            if (ci >= chunks.size()) return;
            const int keepB = chunks[ci].begin, keepE = chunks[ci].end;
            const int b = std::max(0, keepB - kHalf), e = std::min(count, keepE + kHalf);
            const int px = e - b;
            llr.assign(size_t(px) * 10, 0.0f);
            for (int p = 0; p < px; ++p)
                for (int k = 0; k < 10; ++k) {
                    const size_t n = size_t(first + b + p) * 10 + size_t(k);
                    if (n >= nsym) continue;
                    const float v = scale * sym[n];
                    llr[size_t(p) * 10 + size_t(k)] = M[n] ? v : -v;
                }
            softpixel::Vertical vt;
            if (above && sp.vertical) { vt.above = above + b; vt.scale = sp.vScale; vt.edgeFraction = sp.vEdge; }
            const auto r = softpixel::decodeChain(llr.data(), px, 10, cb, -1, sp.prior, vt);
            for (int p = keepB - b; p < keepE - b; ++p) {
                const unsigned v = r.mean[size_t(p)];
                uint16_t& w = words[first + b + p];
                w = uint16_t((v << (10 - cb)) | (w & ((1u << (10 - cb)) - 1u)));
            }
        }
    };
    int nt = sp.threads > 0 ? sp.threads : std::max(1, int(std::thread::hardware_concurrency()) - 1);
    nt = std::min<int>(nt, int(chunks.size()));
    if (nt <= 1) { worker(); return; }
    std::vector<std::thread> pool;
    for (int t = 1; t < nt; ++t) pool.emplace_back([&] { if (sp.threadInit) sp.threadInit(); worker(); });
    worker();
    for (auto& t : pool) t.join();
}

class ImageAssembler {
public:
    using Done = std::function<void(std::shared_ptr<Observation>)>;
    using LineHook = std::function<void(const Observation&, int)>;
    void setLineHook(LineHook h) { hook_ = std::move(h); }
    void setSoft(const SoftParams& p) { soft_ = p; }
    const SoftParams& soft() const { return soft_; }

    void feed(Block&& b, const Done& done) {
        const BlockHeader& h = b.header;
        ++seen_;
        if (!h.plausible) return;
        const bool imager = h.id >= 1 && h.id <= 10 && h.wordSize == 10 && h.crcOk;
        if (h.id == 0 && h.crcOk && h.wordSize == 8) { parseBlock0(b); flushSeries(done); return; }
        if (!imager) return;
        if (b.llr.size() < size_t(16 * 10 + 20)) return;
        if (lastId_ >= h.id) flushSeries(done);
        lastId_ = h.id;
        SeriesBlock sb;
        sb.id = h.id;
        sb.declared = h.wordCount;
        sb.received = int(b.llr.size() / 10);
        // The length that arrived (sync to sync) is measured and reliable; the header's word count can be a CRC repair
        // gone wrong or a template from another frame. Frames differ in length (a sector has 8850-word blocks, the full
        // disk 20841), so no vote across blocks: a vote over the session cut every full-disk block to the sector length.
        int declared = h.wordCount;
        if (!b.truncated && std::abs(sb.received - declared) > 8) { declared = sb.received; sb.declared = declared; ++headersCorrected_; }
        const size_t nwords = std::min<size_t>(b.llr.size() / 10, size_t(declared));
        sb.words.resize(nwords);
        for (size_t w = 0; w < nwords; ++w) {
            unsigned v = 0;
            for (int k = 0; k < 10; ++k) v = (v << 1) | (b.llr[w * 10 + size_t(k)] > 0 ? 1u : 0u);
            sb.words[w] = uint16_t(v);
        }
        sb.doc = parseDoc(sb.words.data());
        sb.docLlr.assign(b.llr.begin(), b.llr.begin() + 16 * 10);
        sb.sym = std::move(b.sym);
        sb.scale = b.llrScale;
        series_.push_back(std::move(sb));
        if (h.id == 10) flushSeries(done);
    }

    // Close the observation in progress (reception stopped).
    void finish(const Done& done) {
        flushSeries(done);
        closeObservation(done);
    }

    uint64_t seriesFlushed() const { return seriesCount_; }
    uint64_t truncatedBlocks() const { return truncatedTotal_; }
    uint64_t softBlocks() const { return softTotal_; }
    uint64_t blocksSeen() const { return seen_; }

private:
    struct SeriesBlock {
        int id = 0;
        std::vector<uint16_t> words;
        LineDoc doc;
        std::vector<int8_t> docLlr;   // soft bits of the 16 line documentation words
        std::vector<float> sym;
        float scale = 0.0f;
        int declared = 0;       // word count in the (CRC-checked) block header
        int received = 0;       // words that actually arrived
    };

    std::map<int, std::map<int, int>> wcVotes_;      // block id -> word count -> blocks seen with it
    std::map<int, int> modalWords_;
    std::map<int, int> recVotes_;                    // IR record words implied by full-length IR blocks
    int modalRec_ = 0;
    uint64_t headersCorrected_ = 0, truncatedTotal_ = 0;

    template <typename T> static T mode(const std::vector<T>& v, T fallback, int* count = nullptr) {
        std::map<T, int> m;
        for (const T& x : v) ++m[x];
        int best = 0; T val = fallback;
        for (auto& kv : m) if (kv.second > best) { best = kv.second; val = kv.first; }
        if (count) *count = best;
        return val;
    }

    void parseBlock0(const Block& b) {
        if (b.llr.size() < 278 * 8) return;
        uint8_t by[278];
        for (int i = 0; i < 278; ++i) {
            unsigned v = 0;
            for (int k = 0; k < 8; ++k) v = (v << 1) | (b.llr[size_t(i) * 8 + size_t(k)] > 0 ? 1u : 0u);
            by[i] = uint8_t(v);
        }
        const auto u16 = [&](int o) { return int((by[o] << 8) | by[o + 1]); };
        const auto bcd = [&](int o) -> std::time_t {
            const auto hi = [&](int i) { return by[o + i] >> 4; };
            const auto lo = [&](int i) { return by[o + i] & 15; };
            const int year = hi(0) * 1000 + lo(0) * 100 + hi(1) * 10 + lo(1);
            const int doy = hi(2) * 100 + lo(2) * 10 + hi(3);
            const int hour = lo(3) * 10 + hi(4), minute = lo(4) * 10 + hi(5);
            int sec = lo(5) * 10 + hi(6);
            if (sec >= 60 && sec - 40 <= 59) sec -= 40;      // BCD borrow artefact of the frame-start time
            if (year < 1990 || year > 2100 || doy < 1 || doy > 366 || hour > 23 || minute > 59 || sec > 59) return 0;
            std::tm t{}; t.tm_year = year - 1900; t.tm_mon = 0; t.tm_mday = 1;
            const std::time_t base = timegm_(t);
            return base + std::time_t(doy - 1) * 86400 + hour * 3600 + minute * 60 + sec;
        };
        const auto bfloat = [&](int o) -> double {   // GVAR 32-bit float: sign, 7 bit base-16 exponent, 24 bit mantissa
            uint32_t m = (uint32_t(by[o]) << 24) | (uint32_t(by[o + 1]) << 16) | (uint32_t(by[o + 2]) << 8) | by[o + 3];
            const bool neg = (m & 0x80000000u) != 0;
            if (neg) m = uint32_t(-int32_t(m));
            const int e = int((m >> 24) & 0x7F) - 64;
            double f = double(m & 0xFFFFFF) / 16777216.0;
            f *= std::pow(16.0, e);
            return neg ? -f : f;
        };
        const std::time_t cifst = bcd(250), tcurr = bcd(22);
        Block0Info bi;
        bi.iwfpx = u16(156); bi.iefpx = u16(158); bi.infln = u16(160); bi.isfln = u16(162); bi.insln = u16(154);
        bi.risct = u16(150); bi.subLat = bfloat(174); bi.subLon = bfloat(178);
        bi.frameStart = cifst; bi.current = tcurr;
        bi.valid = cifst != 0 && bi.iefpx > 0 && bi.isfln > 0;
        if (!bi.valid) return;
        b0votes_.push_back(bi);
    }

    static std::time_t timegm_(std::tm& t) {
#ifdef _WIN32
        return _mkgmtime(&t);
#else
        return ::timegm(&t);
#endif
    }

    void flushSeries(const Done& done) {
        lastId_ = 0;
        if (series_.empty()) return;
        std::vector<SeriesBlock> series = std::move(series_);
        series_.clear();
        ++seriesCount_;
        std::vector<int> counters;
        for (auto& s : series) counters.push_back(s.doc.relScan);
        int votes = 0;
        int counter = mode(counters, -1, &votes);
        // Every block of the line carries the same relative scan count in words 5-6, and it goes up by one per
        // line. Score the likely values on the soft bits of all blocks together (10 blocks x 11 bits), so the
        // count survives symbol error rates at which no single block's hard decision is right.
        const auto score = [&](int c) {
            double sum = 0.0;
            for (auto& s : series) {
                if (s.docLlr.size() < 160) continue;
                const int w5 = (c >> 10) & 1, w6 = c & 0x3FF;
                sum += (w5 ? 1.0 : -1.0) * s.docLlr[5 * 10 + 9];
                for (int j = 0; j < 10; ++j) sum += (((w6 >> (9 - j)) & 1) ? 1.0 : -1.0) * s.docLlr[size_t(6 * 10 + j)];
            }
            return sum;
        };
        // Maximum likelihood over every legal counter, with a prior for continuing the sequence (+1, or +2/+3 after a
        // lost line). Without a previous line (first line, or after a gap) only a clear winner is taken.
        {
            double norm = 0.0;
            for (auto& s : series) {
                if (s.docLlr.size() < 160) continue;
                norm += std::abs(double(s.docLlr[5 * 10 + 9]));
                for (int j = 0; j < 10; ++j) norm += std::abs(double(s.docLlr[size_t(6 * 10 + j)]));
            }
            if (norm <= 0.0) return;
            int best = -1; double top = -1e30, bestRho = -1.0;
            for (int c = 1; c <= kMaxCounter; ++c) {
                const double rho = score(c) / norm;
                double v = rho;
                if (lastCounter_ > 0) {
                    if (c == lastCounter_ + 1) v += 0.15;
                    else if (c == lastCounter_ + 2 || c == lastCounter_ + 3) v += 0.08;
                }
                if (v > top) { top = v; best = c; bestRho = rho; }
            }
            const bool continues = lastCounter_ > 0 && best >= lastCounter_ + 1 && best <= lastCounter_ + 3;
            if (!continues && bestRho < 0.42) {
                // neither the sequence nor a clear value: keep the sequence if there is one, else wait
                if (lastCounter_ > 0 && series.size() >= 4) best = lastCounter_ + 1;
                else return;
            }
            counter = best;
            (void)votes;
        }
        if (counter < 1 || counter > kMaxCounter) return;
        // a counter that fell back to the start means the next frame has begun
        if (obs_ && lastCounter_ > 20 && counter <= 3 && counter < lastCounter_ - 20) closeObservation(done);
        if (!obs_) startObservation();
        lastCounter_ = counter;
        Observation& o = *obs_;
        if (o.firstCounter == 0) o.firstCounter = counter;
        o.lastCounter = counter;
        ++o.lines;
        // widths of this series
        std::vector<int> irRec, visPix;
        for (auto& s : series) {
            if (s.id <= 2) irRec.push_back(s.doc.wordCount);
            else visPix.push_back(s.doc.pixelCount);
        }
        for (auto& s : series) {
            ++o.blocksSeen;
            o.blocksCrcOk++;
            o.headersCorrected = headersCorrected_;
            const int nw = int(s.words.size());
            if (s.received + 8 < s.declared) {            // the information field ended early: do not draw half a row
                ++o.blocksTruncated; ++truncatedTotal_;
                if (s.id >= 0 && s.id <= 10) ++o.truncatedById[s.id];
                if (o.truncatedWordsN < 8) o.truncatedWords[o.truncatedWordsN++] = s.received;
                continue;
            }
            if (s.id >= 3) {
                int npix = std::min(nw - 16 - 2, kVisWidth);
                if (npix <= 0) continue;
                const int y = counter * 8 + (s.id - 3);
                if (soft_.enabled && !s.sym.empty() && s.scale > 0.0f) {
                    const std::vector<uint16_t> above = aboveRow(o.ch[0], y, npix);
                    recoverWords(s.sym.data(), s.sym.size(), s.scale, s.words.data(), 16, npix, soft_,
                                 above.empty() ? nullptr : above.data());
                    ++o.blocksRecovered; ++softTotal_; o.softUsed = true;
                }
                o.visWidthWords = std::max(o.visWidthWords, npix);
                uint16_t* row = o.ch[0].row(y);
                if (!row) continue;
                for (int i = 0; i < npix; ++i) row[i] = uint16_t(s.words[size_t(16 + i)] << 6);
            } else {
                // Blocks 1 and 2 hold four line records, each with its own 16-word line documentation:
                // block 1 = ch2 det 1, ch2 det 2, ch3 det 3, ch3 det 4; block 2 = ch4 det 5, ch4 det 6, ch6 det 7, ch6 det 8.
                // The record length is the documentation's word count (16 + pixels, e.g. 5210 = 16 + 5194), so record r
                // starts at r * stride and its pixels at r * stride + 16. (Reading the four as one run after a single
                // 16-word documentation shifted records 1..3 by 5, 10 and 15 pixels and drew documentation words.)
                int stride = mode(irRec, 0);
                // the four records fill the block: nw = 4 * stride + 1 (the last word is the CRC), in sector scans too
                if (stride <= 16 || nw - 4 * stride < 0 || nw - 4 * stride > 3) {
                    stride = nw / 4;     // the documentation's word count was misread
                    ++o.recOverridden;
                    if (o.diagN < 8) { int* d = o.diag[o.diagN++]; d[0] = counter; d[1] = s.id; d[2] = stride; d[3] = mode(irRec, 0); d[4] = nw; d[5] = s.declared; d[6] = s.doc.wordCount; }
                }
                const int pix = std::min(stride - 16, kIrWidth);
                if (pix <= 0) continue;
                o.irRecordWords = std::max(o.irRecordWords, pix);
                const int base = s.id == 1 ? 1 : 3;
                bool soft = false;
                for (int r = 0; r < 4; ++r) {
                    Raster16& ch = o.ch[base + r / 2];
                    const int y = counter * 2 + (r & 1);
                    const int first = r * stride + 16;
                    if (first + pix > nw) break;
                    {
                        // zeros to the end of the record are not an image (an IR line is never black for 250 words)
                        const int span = 256, edge = 20;
                        bool zero = pix > span + edge + 64;
                        for (int i = pix - edge - span; i < pix - edge && zero; ++i) zero = s.words[size_t(first + i)] == 0;
                        if (zero) { ++o.deadTailRows; continue; }
                    }
                    if (soft_.enabled && !s.sym.empty() && s.scale > 0.0f) {
                        const std::vector<uint16_t> above = aboveRow(ch, y, pix);
                        recoverWords(s.sym.data(), s.sym.size(), s.scale, s.words.data(), first, pix, soft_,
                                     above.empty() ? nullptr : above.data());
                        soft = true;
                    }
                    uint16_t* row = ch.row(y);
                    if (!row) continue;
                    for (int i = 0; i < pix; ++i) row[i] = uint16_t(s.words[size_t(first + i)] << 6);
                }
                if (soft) { ++o.blocksRecovered; ++softTotal_; o.softUsed = true; }
            }
        }
        if (hook_) hook_(o, counter);
    }

    // 10-bit values of row y - 1 when it has been received, else empty.
    std::vector<uint16_t> aboveRow(Raster16& r, int y, int n) const {
        if (!soft_.vertical) return {};
        const uint16_t* p = r.peek(y - 1);
        if (!p) return {};
        std::vector<uint16_t> v(static_cast<size_t>(n), uint16_t(0));
        for (int i = 0; i < n && i < r.width; ++i) v[size_t(i)] = uint16_t(p[i] >> 6);
        return v;
    }

    void startObservation() {
        obs_ = std::make_shared<Observation>();
        obs_->ch[0].init(kVisWidth, kVisRows);
        for (int c = 1; c < 5; ++c) obs_->ch[c].init(kIrWidth, kIrRows);
        b0votes_.clear();
        lastCounter_ = 0;
    }

    void closeObservation(const Done& done) {
        if (!obs_) return;
        auto o = std::move(obs_);
        obs_.reset();
        if (o->lines >= 4) {
            // block 0 majority: frame start time, and the geometry of the most common record
            std::vector<std::time_t> t; std::vector<long long> geo;
            for (auto& v : b0votes_) { t.push_back(v.frameStart); }
            const std::time_t ts = mode(t, std::time_t(0));
            std::map<long long, int> g;
            Block0Info best;
            int bestN = 0;
            for (auto& v : b0votes_) {
                if (v.frameStart != ts) continue;
                const long long key = ((long long)v.iwfpx << 48) ^ ((long long)v.iefpx << 32) ^ ((long long)v.infln << 16) ^ v.isfln;
                if (++g[key] > bestN) { bestN = g[key]; best = v; }
            }
            o->info = best;
            o->info.subLon = mode([&] { std::vector<long long> x; for (auto& v : b0votes_) x.push_back(std::llround(v.subLon * 1000.0)); return x; }(), 0LL) / 1000.0;
            done(o);
        }
        b0votes_.clear();
        lastCounter_ = 0;
    }

    SoftParams soft_;
    LineHook hook_;
    std::vector<SeriesBlock> series_;
    std::vector<Block0Info> b0votes_;
    std::shared_ptr<Observation> obs_;
    int lastId_ = 0, lastCounter_ = 0;
    uint64_t seen_ = 0, seriesCount_ = 0, softTotal_ = 0;
};

}  // namespace nadirstation::gvar
