#pragma once
// GVAR block 0 ("documentation block") field table, decoders and a vote-based consensus.
//
// Block 0 carries 8-bit words; words 1..278 are the fixed scan/frame documentation (SPCID ... PXOR1).
// GVAR has no forward error correction, so with the ~0.3 % raw symbol error rate of a real link a
// 278-byte block normally holds a few wrong bits. Fields that are constant within a frame are
// therefore also reported as the *consensus* of the last N blocks (exact-value plurality per field),
// next to the value of the latest block. Qt-free on purpose; the Qt side only formats and draws.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace nadirstation::gvar {

constexpr int kDocBytes = 278;

enum class DocKind { U8, U16, Float, Bcd, Hex, Bits };

struct DocFieldDef {
    const char* key;      // SPCID, ISCAN ... (names follow the GVAR / SatDump documentation)
    const char* meaning;  // Chinese one-line description
    int off, len;         // byte offset inside the block 0 information field and length
    DocKind kind;
};

inline const std::vector<DocFieldDef>& docFieldDefs() {
    static const std::vector<DocFieldDef> f = {
        {"SPCID", "Spacecraft ID", 0, 1, DocKind::U8},
        {"SPSID", "Processing site ID", 1, 1, DocKind::U8},
        {"ISCAN", "Scan status word (32 bit)", 2, 4, DocKind::Bits},
        {"IDSUB", "Sub-document / spare", 6, 16, DocKind::Hex},
        {"TCURR", "Current time", 22, 8, DocKind::Bcd},
        {"TCHED", "Header time", 30, 8, DocKind::Bcd},
        {"TCTRL", "Trailer time", 38, 8, DocKind::Bcd},
        {"TLHED", "Previous header time", 46, 8, DocKind::Bcd},
        {"TLTRL", "Previous trailer time", 54, 8, DocKind::Bcd},
        {"TIPFS", "Imager first-pixel scan time", 62, 8, DocKind::Bcd},
        {"TINFS", "Imager north-scan time", 70, 8, DocKind::Bcd},
        {"TISPC", "Imager space-clamp time", 78, 8, DocKind::Bcd},
        {"TIECL", "Imager Earth-edge time", 86, 8, DocKind::Bcd},
        {"TIBBC", "Imager black-body cal. time", 94, 8, DocKind::Bcd},
        {"TISTR", "Imager star-sense time", 102, 8, DocKind::Bcd},
        {"TIRAN", "Imager ranging time", 110, 8, DocKind::Bcd},
        {"TIIRT", "Imager IR cal. time", 118, 8, DocKind::Bcd},
        {"TIVIT", "Imager visible cal. time", 126, 8, DocKind::Bcd},
        {"TCLMT", "Clock-error time", 134, 8, DocKind::Bcd},
        {"TIONA", "Orbit/attitude time", 142, 8, DocKind::Bcd},
        {"RISCT", "Relative scan count", 150, 2, DocKind::U16},
        {"AISCT", "Absolute scan count", 152, 2, DocKind::U16},
        {"INSLN", "Current scan line (N-S)", 154, 2, DocKind::U16},
        {"IWFPX", "Frame west-edge pixel", 156, 2, DocKind::U16},
        {"IEFPX", "Frame east-edge pixel", 158, 2, DocKind::U16},
        {"INFLN", "Frame north-edge line", 160, 2, DocKind::U16},
        {"ISFLN", "Frame south-edge line", 162, 2, DocKind::U16},
        {"IMDPX", "Frame centre pixel", 164, 2, DocKind::U16},
        {"IMDLN", "Frame centre line", 166, 2, DocKind::U16},
        {"IMDCT", "Frame centre count", 168, 2, DocKind::U16},
        {"IGVLN", "GVAR line count", 170, 2, DocKind::U16},
        {"IGVPX", "GVAR pixel count", 172, 2, DocKind::U16},
        {"SUBLA", "Sub-satellite latitude (deg)", 174, 4, DocKind::Float},
        {"SUBLO", "Sub-satellite longitude (deg)", 178, 4, DocKind::Float},
        {"CZONE", "Time-zone code", 182, 1, DocKind::U8},
        {"V1PHY", "V1 physical parameter", 183, 1, DocKind::U8},
        {"G1CNT", "G1 count", 184, 2, DocKind::U16},
        {"G2CNT", "G2 count", 186, 2, DocKind::U16},
        {"PBIAS", "Pixel bias", 188, 2, DocKind::U16},
        {"LBIAS", "Line bias", 190, 2, DocKind::U16},
        {"ISCP1", "Scan parameter 1", 192, 2, DocKind::U16},
        {"IDBER", "Data-bit error count", 194, 4, DocKind::Hex},
        {"RANGE", "Range value", 198, 4, DocKind::Float},
        {"GPATH", "Ground-path delay", 202, 4, DocKind::Float},
        {"XMSNE", "Transmission error", 206, 4, DocKind::Hex},
        {"TGPAT", "Ground-path time", 210, 8, DocKind::Bcd},
        {"TXMSN", "Transmission time", 218, 8, DocKind::Hex},
        {"ISTIM", "Scan-time count", 226, 2, DocKind::U16},
        {"IFRAM", "Frame number", 228, 1, DocKind::U8},
        {"IMODE", "Imaging mode", 229, 1, DocKind::U8},
        {"IFNW1", "Frame NW corner 1", 230, 4, DocKind::Float},
        {"IFNW2", "Frame NW corner 2", 234, 4, DocKind::Float},
        {"IFSE1", "Frame SE corner 1", 238, 4, DocKind::Float},
        {"IFSE2", "Frame SE corner 2", 242, 4, DocKind::Float},
        {"IG2TN", "G2 turntable", 246, 1, DocKind::U8},
        {"ISCP2", "Scan parameter 2", 247, 1, DocKind::U8},
        {"ISCA2", "Scan parameter A2", 248, 2, DocKind::U16},
        {"CIFST", "Frame start time", 250, 8, DocKind::Bcd},
        {"SPARE", "Spare", 258, 19, DocKind::Hex},
        {"PXOR1", "Parity byte", 277, 1, DocKind::U8},
    };
    return f;
}

inline int docFieldIndex(const char* key) {
    const auto& f = docFieldDefs();
    for (size_t i = 0; i < f.size(); ++i) if (std::strcmp(f[i].key, key) == 0) return int(i);
    return -1;
}

// ---- decoders -----------------------------------------------------------------------------------

inline double decodeGvarFloat(const uint8_t* p) {
    // sign, 7-bit excess-64 base-16 exponent, 24-bit mantissa; negative values are two's complement
    uint32_t m = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
    const bool neg = (m & 0x80000000u) != 0;
    if (neg) m = uint32_t(-int32_t(m));
    const int e = int((m >> 24) & 0x7F) - 64;
    const double f = double(m & 0xFFFFFF) / 16777216.0 * std::pow(16.0, e);
    return neg ? -f : f;
}

inline unsigned decodeU(const uint8_t* p, int len) {
    unsigned v = 0;
    for (int i = 0; i < len; ++i) v = (v << 8) | p[i];
    return v;
}

struct DocTime {
    bool valid = false;        // every nibble is a decimal digit and the calendar fields are legal
    bool borrowFixed = false;  // seconds were >= 60 and 40 was subtracted (BCD borrow artefact of CIFST)
    int year = 0, doy = 0, hour = 0, minute = 0, second = 0, ms = 0;
    int64_t epochMs = 0;       // UTC, milliseconds
};

inline int64_t daysFromCivil(int y, int m, int d) {
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = unsigned(y - era * 400);
    const unsigned doy = unsigned((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + int64_t(doe) - 719468;
}

// 16 BCD digits, high nibble first: YYYY DDD HH MM SS mmm
inline DocTime decodeBcdTime(const uint8_t* p) {
    DocTime t;
    int d[16];
    for (int i = 0; i < 8; ++i) { d[2 * i] = p[i] >> 4; d[2 * i + 1] = p[i] & 15; }
    for (int i = 0; i < 16; ++i) if (d[i] > 9) return t;
    t.year = d[0] * 1000 + d[1] * 100 + d[2] * 10 + d[3];
    t.doy = d[4] * 100 + d[5] * 10 + d[6];
    t.hour = d[7] * 10 + d[8];
    t.minute = d[9] * 10 + d[10];
    t.second = d[11] * 10 + d[12];
    t.ms = d[13] * 100 + d[14] * 10 + d[15];
    if (t.second >= 60 && t.second - 40 <= 59) { t.second -= 40; t.borrowFixed = true; }
    const bool leap = (t.year % 4 == 0 && t.year % 100 != 0) || t.year % 400 == 0;
    if (t.year < 1990 || t.year > 2100 || t.doy < 1 || t.doy > (leap ? 366 : 365) || t.hour > 23 || t.minute > 59 || t.second > 59) return t;
    t.valid = true;
    t.epochMs = ((daysFromCivil(t.year, 1, 1) + (t.doy - 1)) * 86400 + t.hour * 3600 + t.minute * 60 + t.second) * 1000 + t.ms;
    return t;
}

inline std::string formatTime(const DocTime& t) {
    if (!t.valid) return "—";
    char b[64];
    std::snprintf(b, sizeof b, "%04d-%03d %02d:%02d:%02d.%03d", t.year, t.doy, t.hour, t.minute, t.second, t.ms);
    return b;
}

// Calendar date (UTC) of a time, "YYYY-MM-DD HH:MM:SS.mmm"
inline std::string formatTimeIso(const DocTime& t) {
    if (!t.valid) return "—";
    // civil from days
    int64_t z = t.epochMs / 86400000 + 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = unsigned(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int y = int(yoe) + int(era) * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp < 10 ? mp + 3 : mp - 9;
    y += m <= 2;
    char b[64];
    std::snprintf(b, sizeof b, "%04d-%02u-%02u %02d:%02d:%02d.%03d", y, m, d, t.hour, t.minute, t.second, t.ms);
    return b;
}

inline std::string hexString(const uint8_t* p, int len) {
    std::string s;
    char b[4];
    for (int i = 0; i < len; ++i) { std::snprintf(b, sizeof b, "%02X", p[i]); s += b; if (i + 1 < len) s += ' '; }
    return s;
}

inline const char* imodeName(int m) {
    switch (m) {
        case 1: return "Routine";
        case 2: return "Rapid";
        case 3: return "Super-rapid";
        case 4: return "Checkout";
        default: return "Unknown";
    }
}

// What kind of frame the scan is, from its bounds in visible-detector coordinates (IWFPX, IEFPX, INFLN, ISFLN of
// block 0). GVAR has no field that says so: IMODE is the schedule (routine / rapid ...), and within one schedule the
// imager takes full disks, hemispheres and sectors in turn. The full disk of these imagers is about 20 780 pixels by
// 10 820 lines around the sub-satellite point (GOES-15 example, 2026-11-03: pixels 3997..24772, lines 2485..13304).
inline const char* frameKindName(int west, int east, int north, int south) {
    constexpr double kCentreLine = 7895.0, kCentrePixel = 14385.0;
    constexpr double kHalfLines = 5410.0, kHalfPixels = 10388.0;
    if (west <= 0 || east <= west || north <= 0 || south <= north) return "Unknown";
    const double w = east - west + 1, h = south - north + 1;
    const double nEdge = (kCentreLine - north) / kHalfLines;    // 1 = northern limb, 0 = equator, < 0 south of it
    const double sEdge = (south - kCentreLine) / kHalfLines;    // 1 = southern limb
    const bool wide = w >= 0.8 * 2.0 * kHalfPixels;
    (void)kCentrePixel;
    if (wide && h >= 0.9 * 2.0 * kHalfLines) return "Full disk";
    if (wide && nEdge >= 0.8 && sEdge <= 0.5) return "Northern hemisphere";
    if (wide && sEdge >= 0.8 && nEdge <= 0.5) return "Southern hemisphere";
    return "Sector";
}

// ISCAN as a list of the set bits, numbered 1..32 from the most significant bit (the GVAR convention).
inline std::string iscanBits(const uint8_t* p) {
    std::string s;
    for (int bit = 0; bit < 32; ++bit)
        if (p[bit / 8] & (0x80 >> (bit % 8))) { if (!s.empty()) s += ','; s += std::to_string(bit + 1); }
    return s.empty() ? std::string("none") : s;
}

inline std::string formatField(const DocFieldDef& f, const uint8_t* by) {
    const uint8_t* p = by + f.off;
    char b[96];
    switch (f.kind) {
        case DocKind::U8: std::snprintf(b, sizeof b, "%u", p[0]); return b;
        case DocKind::U16: std::snprintf(b, sizeof b, "%u", decodeU(p, 2)); return b;
        case DocKind::Float: {
            const double v = decodeGvarFloat(p);
            if (!std::isfinite(v)) return "—";
            if (std::fabs(v) >= 1e5) std::snprintf(b, sizeof b, "%.0f", v); else std::snprintf(b, sizeof b, "%.4f", v);
            return b;
        }
        case DocKind::Bcd: { const auto t = decodeBcdTime(p); return t.valid ? formatTime(t) + (t.borrowFixed ? " *" : "") : "invalid " + hexString(p, 8); }
        case DocKind::Bits: return "0x" + [&] { std::string s; char c[4]; for (int i = 0; i < f.len; ++i) { std::snprintf(c, sizeof c, "%02X", p[i]); s += c; } return s; }() + "  bits " + iscanBits(p);
        case DocKind::Hex: default: return hexString(p, f.len);
    }
}

// PXOR1: the XOR of all 278 bytes is 0xFF for an error-free block. An odd number of errors in a bit
// column breaks it; an even number cancels, so "ok" does not prove the block clean.
inline bool docParityOk(const uint8_t* by) {
    uint8_t x = 0;
    for (int i = 0; i < kDocBytes; ++i) x ^= by[i];
    return x == 0xFF;
}

// ---- consensus ----------------------------------------------------------------------------------

struct FieldVote {
    std::array<uint8_t, 19> bytes{};  // most frequent value of the field in the window (largest field is 19 bytes)
    int votes = 0;                    // blocks that carried exactly that value
    int total = 0;                    // blocks in the window
    bool stable() const { return total >= 4 && votes * 2 >= total && votes >= 3; }
};

class DocConsensus {
public:
    static constexpr size_t kWindow = 48;

    void clear() { win_.clear(); }
    size_t size() const { return win_.size(); }

    void add(const uint8_t* by) {
        std::array<uint8_t, kDocBytes> a;
        std::memcpy(a.data(), by, kDocBytes);
        win_.push_back(a);
        if (win_.size() > kWindow) win_.pop_front();
    }

    FieldVote vote(size_t field) const {
        FieldVote v;
        const auto& fd = docFieldDefs()[field];
        v.total = int(win_.size());
        if (win_.empty()) return v;
        for (size_t i = 0; i < win_.size(); ++i) {
            int n = 0;
            for (size_t j = 0; j < win_.size(); ++j)
                if (std::memcmp(win_[i].data() + fd.off, win_[j].data() + fd.off, size_t(fd.len)) == 0) ++n;
            if (n > v.votes) { v.votes = n; std::memcpy(v.bytes.data(), win_[i].data() + fd.off, size_t(fd.len)); }
        }
        return v;
    }

    // The block assembled from the per-field winners (fields without a stable winner keep the newest value).
    std::array<uint8_t, kDocBytes> consensusBlock(std::vector<FieldVote>* votes = nullptr) const {
        std::array<uint8_t, kDocBytes> out{};
        if (!win_.empty()) out = win_.back();
        if (votes) votes->clear();
        const auto& f = docFieldDefs();
        for (size_t i = 0; i < f.size(); ++i) {
            const FieldVote v = vote(i);
            if (v.stable()) std::memcpy(out.data() + f[i].off, v.bytes.data(), size_t(f[i].len));
            if (votes) votes->push_back(v);
        }
        return out;
    }

    // Differing bits between a block and the consensus, counted over the stable fields only. bits = how many were compared.
    int stableErrorBits(const uint8_t* by, int* bits = nullptr) const {
        int errs = 0, cnt = 0;
        const auto& f = docFieldDefs();
        for (size_t i = 0; i < f.size(); ++i) {
            if (!std::strcmp(f[i].key, "SPARE")) continue;
            const FieldVote v = vote(i);
            if (!v.stable()) continue;
            for (int k = 0; k < f[i].len; ++k) {
                unsigned x = unsigned(by[f[i].off + k] ^ v.bytes[size_t(k)]);
                while (x) { errs += int(x & 1u); x >>= 1; }
            }
            cnt += f[i].len * 8;
        }
        if (bits) *bits = cnt;
        return errs;
    }

private:
    std::deque<std::array<uint8_t, kDocBytes>> win_;
};

}  // namespace nadirstation::gvar
