#include "gvarengine.h"
#include "runtime_logger.h"
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimeZone>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <future>
#include <thread>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace nadirstation::gvar {
namespace {

// 256 x 256 RGB, row = IR 10.7 um, column = visible: SatDump's GOES GVAR false-colour table.
const unsigned char kFalseColorLut[256 * 256 * 3] = {
#include "gvar_fc_lut.inc"
};

constexpr int kPreviewWidth = 1300;       // 16 visible pixels per column
constexpr int kVertStretch100 = 175;      // SatDump stretches every GVAR channel 1.75x vertically

struct ChannelName { const char* file; const char* label; double micron; };
const ChannelName kChannels[5] = {
    {"GVAR_VIS_0p65um.png", "VIS 0.65 um", 0.65},
    {"GVAR_IR2_3p9um.png", "IR2 3.9 um", 3.9},
    {"GVAR_IR3_6p5um.png", "IR3 6.5 um (water vapour)", 6.55},
    {"GVAR_IR4_10p7um.png", "IR4 10.7 um", 10.7},
    {"GVAR_IR5_13p3um.png", "IR5 13.3 um", 13.35},
};

QString stampOf(const Observation& o) {
    const qint64 t = o.info.frameStart > 0 ? qint64(o.info.frameStart) : QDateTime::currentSecsSinceEpoch();
    return QDateTime::fromSecsSinceEpoch(t, QTimeZone::UTC).toString(QStringLiteral("yyyyMMdd_HHmmss"));
}

// Fill the rows of [y0, y1] that never arrived from the nearest received rows above and below.
void fillMissing(Raster16& r, int y0, int y1, int w) {
    int prev = -1;
    for (int y = y0; y <= y1; ++y) {
        if (r.peek(y)) { prev = y; continue; }
        int next = -1;
        for (int k = y + 1; k <= y1; ++k) if (r.peek(k)) { next = k; break; }
        if (prev < 0 && next < 0) return;
        uint16_t* dst = r.row(y);
        const uint16_t* a = prev >= 0 ? r.peek(prev) : nullptr;
        const uint16_t* b = next >= 0 ? r.peek(next) : nullptr;
        for (int x = 0; x < w; ++x)
            dst[x] = (a && b) ? uint16_t((unsigned(a[x]) + unsigned(b[x])) / 2) : (a ? a[x] : b[x]);
    }
}

// Rows [y0, y0 + h) x width w of a raster as a 16-bit image; source rows are released as they are copied.
QImage rasterToImage(Raster16& r, int y0, int h, int w) {
    QImage img(w, h, QImage::Format_Grayscale16);
    if (img.isNull()) return img;
    for (int y = 0; y < h; ++y) {
        const uint16_t* src = r.peek(y0 + y);
        if (src) std::memcpy(img.scanLine(y), src, size_t(w) * 2);
        else std::memset(img.scanLine(y), 0, size_t(w) * 2);
        r.rows[size_t(y0 + y)].reset();
    }
    return img;
}

// The same rows stretched vertically by stretch100 / 100 (a GVAR scan line is 1.75 times closer to its neighbour than a
// pixel is to its own, so the 1:1 image looks squashed): linear blend between the two nearest source rows. Source rows
// are released as soon as no later output row needs them, so the unstretched image is never held in memory.
QImage rasterToStretchedImage(Raster16& r, int y0, int h, int w, int stretch100) {
    const int outH = int((int64_t(h) * stretch100) / 100);
    QImage img(w, outH, QImage::Format_Grayscale16);
    if (img.isNull()) return img;
    int released = 0;
    for (int y = 0; y < outH; ++y) {
        const int64_t pos = int64_t(y) * 10000 / stretch100;       // source row in hundredths
        const int i0 = int(std::min<int64_t>(pos / 100, h - 1)), i1 = std::min(i0 + 1, h - 1);
        const int f = int(pos % 100);
        const uint16_t* a = r.peek(y0 + i0);
        const uint16_t* b = r.peek(y0 + i1);
        uint16_t* dst = reinterpret_cast<uint16_t*>(img.scanLine(y));
        if (!a && !b) std::memset(dst, 0, size_t(w) * 2);
        else if (!a || !b || f == 0 || i0 == i1) std::memcpy(dst, a ? a : b, size_t(w) * 2);
        else for (int x = 0; x < w; ++x) dst[x] = uint16_t((unsigned(a[x]) * unsigned(100 - f) + unsigned(b[x]) * unsigned(f) + 50u) / 100u);
        for (; released < i0; ++released) r.rows[size_t(y0 + released)].reset();
    }
    for (; released < h; ++released) r.rows[size_t(y0 + released)].reset();
    return img;
}

}  // namespace

Engine::Engine() {
    // filter-first demodulation: the matched filter runs in front(), which process() hands to another core
    demod_.setFilterFirst(true);
    deframer_.setKeepSymbols(true);
    deframer_.setHeaderPrediction(true);      // keep scan lines whose header CRC fails near the SNR threshold
    assemblerCore_.setLineHook([this](const Observation& o, int counter) { onLine(o, counter); });
    assembler_ = std::thread([this] { assemblerLoop(); });
    writer_ = std::thread([this] { writerLoop(); });
}

Engine::~Engine() {
    if (deframeJob_.valid()) deframeJob_.get();
    {
        std::lock_guard<std::mutex> l(qMutex_);
        qStop_ = true;
    }
    qCv_.notify_all();
    if (assembler_.joinable()) assembler_.join();
    {
        std::lock_guard<std::mutex> l(wMutex_);
        wStop_ = true;
    }
    wCv_.notify_all();
    if (writer_.joinable()) writer_.join();
}

bool Engine::configure(double sampleRate, double symbolRate, float rrcAlpha, float loopBandwidth) {
    sampleRate_ = sampleRate;
    symbolRate_ = symbolRate;
    // GVAR is a continuous, stable carrier: a narrow carrier loop holds lock about 1.5 dB lower than the
    // 0.02 of the generic setting (measured on a real recording with added noise), and costs nothing at normal SNR.
    loopBandwidth = std::min(loopBandwidth, 0.003f);
    // The PSK chain works from 1.3 samples a symbol upward; 2.88 MS/s (1.36) was decoded in testing.
    if (sampleRate < 1.3 * symbolRate) { configured_ = false; return false; }
    if (deframeJob_.valid()) deframeJob_.get();
    pendingFront_.clear();                     // filtered at the old rate
    demod_.configure(float(sampleRate), float(symbolRate), rrcAlpha, loopBandwidth);
    configured_ = true;
    return true;
}

void Engine::setOutputRoot(const QString& root) { outputRoot_ = root; }

int Engine::queuedBlocks() const {
    std::lock_guard<std::mutex> l(qMutex_);
    return int(blocks_.size());
}

void Engine::reset() {
    // Anything still in flight belongs to the previous reception.
    finish();
    deframer_ = Deframer();
    deframer_.setKeepSymbols(true);
    deframer_.setHeaderPrediction(true);      // keep scan lines whose header CRC fails near the SNR threshold
    demod_.reset();
    pendingFront_.clear();
    snrY1_ = snrY2_ = 0.0;
    lock_ = 0.f; carrierHz_ = 0.f; snr_ = 0.0;
    blocksGood_ = blocksBad_ = syncs_ = lines_ = softBlocks_ = softSkipped_ = 0;
    lastBlocks_ = 0;
    {
        std::lock_guard<std::mutex> l(tMutex_);
        doc_.clear();
        block0Raw_.clear();
        products_.clear();
        csvPending_.clear();
        blocks0_ = parityOk_ = telemetrySeen_ = telemetryTaken_ = savedBlocks0_ = 0;
        lastErrBits_ = lastCompared_ = 0;
        telemetryDir_.clear();
        lastTelemetrySaveMs_ = 0;
    }
    {
        std::lock_guard<std::mutex> l(uMutex_);
        live_ = Update();
        completed_.clear();
        preview_ = QImage();
        previewFirst_ = previewLast_ = -1;
    }
}

dsp::DPSKResult Engine::process(const std::vector<std::complex<float>>& iq) {
    dsp::DPSKResult d;
    if (!configured_ || iq.empty()) return d;
    // Three cores in a pipeline, each stage on its own block: the filter stage (upsampling, AGC, matched filter)
    // of this block on a helper thread, the carrier and timing loops of the previous block on this thread, and
    // block sync / headers of the block before that on another helper. The stages share no state, so the output
    // is the same as running them one after the other, a few tens of milliseconds later.
    auto front = std::async(std::launch::async, [this, &iq] { return demod_.frontStage(iq); });
    if (!pendingFront_.empty()) d = demod_.backStage(pendingFront_);
    if (!d.symbols.empty()) {
        carrierHz_ = d.carrierOffsetHz;
        lock_ = demod_.lockMeasure();
    }
    if (deframeJob_.valid()) deframeJob_.get();
    if (!d.symbols.empty())
        deframeJob_ = std::async(std::launch::async, [this, syms = d.symbols] { handleSymbols(syms); });
    pendingFront_ = front.get();
    return d;
}

void Engine::handleSymbols(const std::vector<std::complex<float>>& symbols) {
    if (symbols.empty()) return;
    std::vector<float> s(symbols.size());
    for (size_t i = 0; i < s.size(); ++i) s[i] = symbols[i].real();
    deframer_.feed(s.data(), s.size(), [this](Block&& b) {
        // a header recovered from the block sequence (guessed) counts as received, block 0/11 included
        if (b.header.crcOk || b.header.guessed) ++blocksGood_; else ++blocksBad_;
        const int id = b.header.id;
        if (b.header.crcOk && (id == 0 || (id > 10 && id < 15))) collectTelemetry(b);
        if (!b.header.crcOk || id > 10 || id < 0) return;       // the assembler only takes block 0 and the imager blocks
        if (id == 0) { b.sym.clear(); b.sym.shrink_to_fit(); }
        std::lock_guard<std::mutex> l(qMutex_);
        // Bounded: at the limit the oldest imager block goes (the series it belongs to is lost, and the
        // image fills that line from its neighbours); this is only reached if the assembler is stuck.
        while (blocks_.size() >= 48) blocks_.pop_front();
        blocks_.push_back(std::move(b));
        qCv_.notify_one();
    });
    syncs_ = deframer_.stats().syncs;
    // SNR the way SatDump reports it: M2M4 over all symbols, alpha = 0.001.
    {
        constexpr double alpha = 0.001, beta = 1.0 - alpha;
        double y1 = snrY1_, y2 = snrY2_;
        for (const auto& z : symbols) {
            const double m = double(z.real()) * z.real() + double(z.imag()) * z.imag();
            y1 = alpha * m + beta * y1;
            y2 = alpha * m * m + beta * y2;
        }
        snrY1_ = std::isfinite(y1) ? y1 : 0.0;
        snrY2_ = std::isfinite(y2) ? y2 : 0.0;
        const double signal = std::sqrt(std::max(0.0, 2.0 * snrY1_ * snrY1_ - snrY2_));
        const double noise = snrY1_ - signal;
        snr_ = (signal > 0.0 && noise > 1e-12) ? std::max(0.0, 10.0 * std::log10(signal / noise)) : 0.0;
    }
}

Engine::Update Engine::takeLive() {
    std::lock_guard<std::mutex> l(uMutex_);
    Update u = std::move(live_);
    live_ = Update();
    return u;
}

std::vector<Engine::Update> Engine::takeCompleted() {
    std::lock_guard<std::mutex> l(uMutex_);
    std::vector<Update> out;
    out.swap(completed_);
    return out;
}

// ---- telemetry ----------------------------------------------------------------------------------

void Engine::collectTelemetry(const Block& b) {
    const int ws = b.header.wordSize;
    const size_t words = std::min<size_t>(b.llr.size() / size_t(std::max(1, ws)), size_t(std::max(0, b.header.wordCount)));
    if (ws <= 0 || words == 0) return;
    const size_t bits = words * size_t(ws);
    std::vector<uint8_t> bytes((bits + 7) / 8, 0);
    for (size_t i = 0; i < bits; ++i)
        if (b.llr[i] > 0) bytes[i >> 3] |= uint8_t(0x80u >> (i & 7));
    std::lock_guard<std::mutex> l(tMutex_);
    if (b.header.id == 0) {
        if (ws != 8 || bytes.size() < size_t(kDocBytes)) return;
        const bool parity = docParityOk(bytes.data());
        ++blocks0_;
        if (parity) ++parityOk_;
        doc_.add(bytes.data());
        lastErrBits_ = doc_.stableErrorBits(bytes.data(), &lastCompared_);
        block0Raw_ = bytes;
        // one row per scan for the CSV
        const auto field = [&](const char* key) { return formatField(docFieldDefs()[size_t(docFieldIndex(key))], bytes.data()); };
        std::string row = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")).toStdString();
        for (const char* k : {"TCURR", "CIFST", "RISCT", "AISCT", "INSLN", "ISTIM", "IFRAM", "IMODE", "SPCID"}) row += "," + field(k);
        row += "," + hexString(bytes.data() + 2, 4);
        row += parity ? ",1" : ",0";
        row += "," + std::to_string(lastErrBits_) + "," + std::to_string(lastCompared_);
        csvPending_.push_back(std::move(row));
        if (csvPending_.size() > 20000) csvPending_.erase(csvPending_.begin(), csvPending_.begin() + 10000);
        ++telemetrySeen_;
        return;
    }
    TelemetryProduct* p = nullptr;
    for (auto& e : products_) if (e.blockId == b.header.id && e.productId == b.header.productId) { p = &e; break; }
    if (!p) {
        products_.emplace_back();
        p = &products_.back();
        p->blockId = b.header.id;
        p->productId = b.header.productId;
    }
    p->wordSize = ws;
    p->wordCount = int(words);
    ++p->count;
    p->bytes = std::move(bytes);
    ++telemetrySeen_;
}

bool Engine::takeTelemetry(Telemetry& out) {
    std::lock_guard<std::mutex> l(tMutex_);
    if (telemetrySeen_ == telemetryTaken_) return false;
    telemetryTaken_ = telemetrySeen_;
    out.block0 = block0Raw_;
    out.window = int(doc_.size());
    out.blocks0 = blocks0_;
    out.parityOk = parityOk_;
    out.errBits = lastErrBits_;
    out.comparedBits = lastCompared_;
    out.products = products_;
    out.votes.clear();
    if (doc_.size() > 0) {
        std::vector<FieldVote> votes;
        out.consensus = doc_.consensusBlock(&votes);
        for (const auto& v : votes) out.votes.push_back(v.votes);
    }
    return true;
}

void Engine::saveTelemetry(bool force) {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    Telemetry t;
    std::vector<std::string> csv;
    QString dir;
    {
        std::lock_guard<std::mutex> l(tMutex_);
        if (blocks0_ == 0 && products_.empty()) return;
        if (!force && lastTelemetrySaveMs_ != 0 && now - lastTelemetrySaveMs_ < 5000) return;
        if (blocks0_ == savedBlocks0_ && !force && lastTelemetrySaveMs_ != 0) return;
        lastTelemetrySaveMs_ = now;
        savedBlocks0_ = blocks0_;
        if (telemetryDir_.isEmpty())
            telemetryDir_ = QDir(outputRoot_).filePath(QStringLiteral("GOES GVAR/Telemetry_") + QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd_HHmmss")));
        dir = telemetryDir_;
        t.block0 = block0Raw_;
        t.window = int(doc_.size());
        t.blocks0 = blocks0_;
        t.parityOk = parityOk_;
        t.errBits = lastErrBits_;
        t.comparedBits = lastCompared_;
        t.products = products_;
        if (doc_.size() > 0) {
            std::vector<FieldVote> votes;
            t.consensus = doc_.consensusBlock(&votes);
            for (const auto& v : votes) t.votes.push_back(v.votes);
        }
        csv.swap(csvPending_);
    }
    if (outputRoot_.isEmpty() || !QDir().mkpath(QDir(dir).filePath(QStringLiteral("Products")))) return;
    const auto writeFile = [&](const QString& name, const void* data, qint64 n) {
        QFile f(QDir(dir).filePath(name));
        if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) f.write(static_cast<const char*>(data), n);
    };
    if (!t.block0.empty()) writeFile(QStringLiteral("Block0_latest.bin"), t.block0.data(), qint64(t.block0.size()));
    if (t.window > 0) writeFile(QStringLiteral("Block0_consensus_278.bin"), t.consensus.data(), qint64(t.consensus.size()));
    for (const auto& p : t.products)
        writeFile(QStringLiteral("Products/Block%1_Product%2_latest.bin").arg(p.blockId).arg(p.productId), p.bytes.data(), qint64(p.bytes.size()));
    if (!csv.empty()) {
        QFile f(QDir(dir).filePath(QStringLiteral("Block0_log.csv")));
        const bool fresh = !f.exists();
        if (f.open(QIODevice::WriteOnly | QIODevice::Append)) {
            if (fresh) f.write("utc_received,TCURR,CIFST,RISCT,AISCT,INSLN,ISTIM,IFRAM,IMODE,SPCID,ISCAN_hex,parity_ok,bit_errors_vs_consensus,bits_compared\n");
            for (const auto& r : csv) { f.write(r.data(), qint64(r.size())); f.write("\n", 1); }
        }
    }
    QJsonObject root;
    root[QStringLiteral("blocks0_received")] = qint64(t.blocks0);
    root[QStringLiteral("blocks0_parity_ok")] = qint64(t.parityOk);
    root[QStringLiteral("consensus_window")] = t.window;
    QJsonArray fields;
    if (!t.block0.empty() && t.block0.size() >= size_t(kDocBytes)) {
        const auto& defs = docFieldDefs();
        for (size_t i = 0; i < defs.size(); ++i) {
            QJsonObject f;
            f[QStringLiteral("key")] = QString::fromLatin1(defs[i].key);
            f[QStringLiteral("meaning")] = QString::fromUtf8(defs[i].meaning);
            f[QStringLiteral("latest")] = QString::fromStdString(formatField(defs[i], t.block0.data()));
            if (t.window > 0 && i < t.votes.size()) {
                f[QStringLiteral("consensus")] = QString::fromStdString(formatField(defs[i], t.consensus.data()));
                f[QStringLiteral("votes")] = t.votes[i];
            }
            fields.append(f);
        }
    }
    root[QStringLiteral("block0_fields")] = fields;
    QJsonArray products;
    for (const auto& p : t.products) {
        QJsonObject o;
        o[QStringLiteral("block_id")] = p.blockId;
        o[QStringLiteral("product_id")] = p.productId;
        o[QStringLiteral("word_size_bits")] = p.wordSize;
        o[QStringLiteral("words")] = p.wordCount;
        o[QStringLiteral("blocks_received")] = qint64(p.count);
        products.append(o);
    }
    root[QStringLiteral("other_blocks")] = products;
    const QByteArray json = QJsonDocument(root).toJson(QJsonDocument::Indented);
    writeFile(QStringLiteral("GVAR_Telemetry.json"), json.constData(), json.size());
}

void Engine::finish() {
    // the last filtered block still waiting for the loop stage
    if (deframeJob_.valid()) deframeJob_.get();
    if (!pendingFront_.empty()) {
        const dsp::DPSKResult d = demod_.backStage(pendingFront_);
        pendingFront_.clear();
        handleSymbols(d.symbols);
    }
    // The block in progress and the symbols still inside the deframer.
    deframer_.flush([this](Block&& b) {
        if (b.header.crcOk && (b.header.id == 0 || (b.header.id > 10 && b.header.id < 15))) collectTelemetry(b);
        if (!b.header.crcOk || b.header.id > 10 || b.header.id < 0) return;
        if (b.header.id == 0) b.sym.clear();
        std::lock_guard<std::mutex> l(qMutex_);
        blocks_.push_back(std::move(b));
    });
    {
        std::lock_guard<std::mutex> l(qMutex_);
        qFinish_ = true;
    }
    qCv_.notify_all();
    {
        std::unique_lock<std::mutex> l(qMutex_);
        qIdleCv_.wait(l, [this] { return blocks_.empty() && !qFinish_ && !qBusy_; });
    }
    {
        std::unique_lock<std::mutex> l(wMutex_);
        wIdleCv_.wait(l, [this] { return toWrite_.empty() && !wBusy_; });
    }
    saveTelemetry(true);
}

// ---- assembler thread --------------------------------------------------------------------------

void Engine::queueObservation(std::shared_ptr<Observation> o) {
    {
        std::lock_guard<std::mutex> l(wMutex_);
        toWrite_.push_back(std::move(o));
    }
    wCv_.notify_one();
}

void Engine::assemblerLoop() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    const int hw = int(std::thread::hardware_concurrency());
    const auto done = [this](std::shared_ptr<Observation> o) { queueObservation(std::move(o)); };
    for (;;) {
        Block b;
        bool haveBlock = false, doFinish = false;
        size_t depth = 0;
        {
            std::unique_lock<std::mutex> l(qMutex_);
            qCv_.wait(l, [this] { return qStop_ || !blocks_.empty() || qFinish_; });
            if (!blocks_.empty()) {
                b = std::move(blocks_.front());
                blocks_.pop_front();
                depth = blocks_.size();
                haveBlock = true;
                qBusy_ = true;
            } else if (qFinish_) {
                qFinish_ = false;
                doFinish = true;
                qBusy_ = true;
            } else if (qStop_) {
                return;
            }
        }
        if (haveBlock) {
            SoftParams sp = assemblerCore_.soft();
            // Soft recovery is the expensive part. With a backlog it is skipped for this block, so the
            // queue drains and no line is lost; the picture of that line keeps its hard decisions.
            sp.enabled = softWanted_.load() && depth < 12;
            // leave three cores to the receiver, the filter stage and the loop stage; the workers run below normal priority
            sp.threads = std::max(1, hw - 3);
#ifdef _WIN32
            sp.threadInit = [] { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL); };
#endif
            if (softWanted_.load() && !sp.enabled) ++softSkipped_;
            assemblerCore_.setSoft(sp);
            const uint64_t before = assemblerCore_.seriesFlushed();
            try { assemblerCore_.feed(std::move(b), done); }
            catch (const std::exception& e) { nadirstation::log::write(QStringLiteral("GVAR assembler: %1").arg(QString::fromUtf8(e.what()))); }
            if (assemblerCore_.seriesFlushed() != before) lines_ = assemblerCore_.seriesFlushed();
            softBlocks_ = assemblerCore_.softBlocks();
        }
        if (doFinish) {
            try { assemblerCore_.finish(done); }
            catch (const std::exception& e) { nadirstation::log::write(QStringLiteral("GVAR assembler: %1").arg(QString::fromUtf8(e.what()))); }
            std::lock_guard<std::mutex> l(uMutex_);
            preview_ = QImage();
            previewFirst_ = previewLast_ = -1;
        }
        {
            std::lock_guard<std::mutex> l(qMutex_);
            qBusy_ = false;
        }
        qIdleCv_.notify_all();
    }
}

// Called by the assembler after every stored scan line (one per second): update the live visible view.
void Engine::onLine(const Observation& o, int counter) {
    if (counter < 1 || counter > kMaxCounter) return;
    std::lock_guard<std::mutex> l(uMutex_);
    if (preview_.isNull() || counter < previewLast_ - 20) {
        preview_ = QImage(kPreviewWidth, kMaxCounter, QImage::Format_Grayscale8);
        if (preview_.isNull()) return;
        preview_.fill(0);
        previewFirst_ = previewLast_ = -1;
        previewStamp_ = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    }
    const int width = std::max(16, std::min(o.visWidthWords, kPreviewWidth * 16));
    uint8_t* dst = preview_.scanLine(counter - 1);
    for (int x = 0; x < kPreviewWidth; ++x) {
        const int x0 = x * 16;
        if (x0 >= width) { dst[x] = 0; continue; }
        const int x1 = std::min(width, x0 + 16);
        unsigned sum = 0, n = 0;
        for (int d = 0; d < 8; ++d) {
            const uint16_t* r = o.ch[0].peek(counter * 8 + d);
            if (!r) continue;
            for (int k = x0; k < x1; ++k) sum += r[k];
            n += unsigned(x1 - x0);
        }
        dst[x] = n ? uint8_t(std::min<unsigned>(255, (sum / n) >> 8)) : 0;
    }
    previewFirst_ = previewFirst_ < 0 ? counter : std::min(previewFirst_, counter);
    previewLast_ = std::max(previewLast_, counter);

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (now - lastLiveMs_ < 900) return;
    lastLiveMs_ = now;
    const int rows = previewLast_;                       // from the first row of the canvas: unreceived rows stay black
    const int top = std::max(0, previewFirst_ - 1);
    QImage live = preview_.copy(0, top, kPreviewWidth, rows - top);
    // Percentile stretch over the received pixels so a night-side or hazy disk is still readable.
    uint32_t hist[256] = {};
    uint64_t total = 0;
    for (int y = 0; y < live.height(); ++y) {
        const uint8_t* p = live.constScanLine(y);
        for (int x = 0; x < live.width(); ++x) if (p[x]) { ++hist[p[x]]; ++total; }
    }
    if (total > 2000) {
        uint64_t acc = 0; int lo = 1, hi = 255;
        for (int v = 1; v < 256; ++v) { acc += hist[v]; if (acc >= total / 100) { lo = v; break; } }
        acc = 0;
        for (int v = 1; v < 256; ++v) { acc += hist[v]; if (acc >= total - total / 200) { hi = v; break; } }
        if (hi < lo + 8) hi = std::min(255, lo + 8);
        uint8_t lut[256];
        for (int v = 0; v < 256; ++v) lut[v] = v == 0 ? 0 : uint8_t(std::clamp((v - lo) * 255 / std::max(1, hi - lo), 1, 255));
        for (int y = 0; y < live.height(); ++y) { uint8_t* p = live.scanLine(y); for (int x = 0; x < live.width(); ++x) p[x] = lut[p[x]]; }
    }
    live_.live = std::move(live);
    live_.stamp = previewStamp_;
    live_.message = QStringLiteral("GOES GVAR · %1 lines received (%2/%3) · %4 soft-decision blocks")
        .arg(o.lines).arg(counter).arg(kMaxCounter).arg(qulonglong(o.blocksRecovered));
}

// ---- writer thread -----------------------------------------------------------------------------

void Engine::writerLoop() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    for (;;) {
        std::shared_ptr<Observation> o;
        {
            std::unique_lock<std::mutex> l(wMutex_);
            wCv_.wait(l, [this] { return wStop_ || !toWrite_.empty(); });
            if (toWrite_.empty()) { if (wStop_) return; continue; }
            o = std::move(toWrite_.front());
            toWrite_.pop_front();
            wBusy_ = true;
        }
        try { writeObservation(o); }
        catch (const std::exception& e) {
            nadirstation::log::write(QStringLiteral("GVAR writer: %1").arg(QString::fromUtf8(e.what())));
        }
        o.reset();
        {
            std::lock_guard<std::mutex> l(wMutex_);
            wBusy_ = false;
        }
        wIdleCv_.notify_all();
    }
}

void Engine::writeObservation(const std::shared_ptr<Observation>& op) {
    Observation& o = *op;
    Update update;
    update.complete = true;
    const QString dirStamp = stampOf(o);
    update.stamp = QDateTime::fromSecsSinceEpoch(o.info.frameStart > 0 ? qint64(o.info.frameStart) : QDateTime::currentSecsSinceEpoch(), QTimeZone::UTC)
                       .toString(QStringLiteral("yyyy-MM-dd HH:mm 'UTC'"));
    const QString dir = QDir(outputRoot_).filePath(QStringLiteral("GOES GVAR/") + dirStamp);
    if (!QDir().mkpath(dir)) {
        update.message = QStringLiteral("GOES GVAR: cannot create output folder %1").arg(dir);
        std::lock_guard<std::mutex> l(uMutex_);
        completed_.push_back(update);
        return;
    }
    {
        QString cut, diagText;
        for (int i = 0; i < o.diagN; ++i) {
            const int* d = o.diag[i];
            diagText += QStringLiteral(" [line %1 block %2: record %3 (found %4), block words %5/%6, doc words %7]")
                            .arg(d[0]).arg(d[1]).arg(d[2]).arg(d[3]).arg(d[4]).arg(d[5]).arg(d[6]);
        }
        for (int i = 0; i < o.truncatedWordsN; ++i) cut += (i ? QStringLiteral(",") : QString()) + QString::number(o.truncatedWords[i]);
        nadirstation::log::write(QStringLiteral("GVAR writing observation %1: %2 lines, blocks %3 soft %4, truncated blocks skipped %5 (ids 1/2: %6/%7, first lengths in words: %8), header lengths corrected %9, IR record length overridden %10, dead-tail IR rows skipped %11%12")
            .arg(dirStamp).arg(o.lines).arg(qulonglong(o.blocksSeen)).arg(qulonglong(o.blocksRecovered))
            .arg(qulonglong(o.blocksTruncated)).arg(o.truncatedById[1]).arg(o.truncatedById[2]).arg(cut).arg(qulonglong(o.headersCorrected))
            .arg(qulonglong(o.recOverridden)).arg(qulonglong(o.deadTailRows)).arg(diagText));
    }

    const int c0 = o.firstCounter, c1 = o.lastCounter;
    int visW = o.visWidthWords;
    if (o.info.valid) {
        const int xs = std::abs(o.info.iefpx - o.info.iwfpx) + 1;
        if (xs > 0) visW = std::min(visW, xs);
    }
    int irW = std::min(o.irRecordWords, visW / 4);
    if (irW <= 0) irW = o.irRecordWords;
    const int visY0 = c0 * 8, visH = (c1 - c0 + 1) * 8;
    const int irY0 = c0 * 2, irH = (c1 - c0 + 1) * 2;
    const bool haveVis = visW > 0 && o.ch[0].firstRow() >= 0;
    const bool haveIr4 = irW > 0 && o.ch[3].firstRow() >= 0;

    if (haveVis) fillMissing(o.ch[0], visY0, visY0 + visH - 1, visW);
    for (int c = 1; c < 5; ++c) if (irW > 0 && o.ch[c].firstRow() >= 0) fillMissing(o.ch[c], irY0, irY0 + irH - 1, irW);

    QStringList paths;
    QJsonArray channelsJson;

    // ---- false colour: SatDump's lookup table, visible (x) against IR 10.7 (y), at the VISIBLE resolution (1 km,
    // like SatDump: the 4 km IR channel is interpolated up to the visible grid, not the visible averaged down),
    // with the same 1.75x vertical stretch SatDump applies to every GVAR channel. A full disk at that size is
    // 20944 x 18956 pixels (1.2 GB as RGB); above kFcMaxPixels the picture is made at half the visible resolution.
    if (haveVis && haveIr4) {
        constexpr double kFcMaxPixels = 260e6;
        const int scale = double(visW) * double(visH) * kVertStretch100 / 100.0 > kFcMaxPixels ? 2 : 1;
        const int fcW = visW / scale;
        const int fcH = visH * kVertStretch100 / 100 / scale;
        QImage fc(fcW, fcH, QImage::Format_RGB888);
        if (!fc.isNull()) {
            // IR sample position of each output column: visible pixel x covers IR pixel (x + 0.5) / 4 - 0.5
            std::vector<int> ix0(static_cast<std::size_t>(fcW), 0), ix1(static_cast<std::size_t>(fcW), 0);
            std::vector<uint16_t> wx(static_cast<std::size_t>(fcW), 0);        // weight of ix1, 0..256
            for (int x = 0; x < fcW; ++x) {
                const double vx = (x + 0.5) * scale - 0.5;
                const double f = std::clamp((vx + 0.5) / 4.0 - 0.5, 0.0, double(irW - 1));
                const int a = int(f);
                ix0[size_t(x)] = a; ix1[size_t(x)] = std::min(a + 1, irW - 1);
                wx[size_t(x)] = uint16_t(std::lround((f - a) * 256.0));
            }
            uchar* const bits = fc.bits();             // taken once: scanLine() from several threads would each try to detach
            const qsizetype bpl = fc.bytesPerLine();
            const auto toByte = [](unsigned v16) { return std::min(255u, (v16 * 255u + 32767u) / 65535u); };
            const auto work = [&](int yBegin, int yEnd) {
                for (int y = yBegin; y < yEnd; ++y) {
                    // visible row behind output row y (vertical stretch, then the half-resolution step)
                    const int vy = std::min(visH - 1, int((long long)(y * scale) * 100 / kVertStretch100));
                    const double fy = std::clamp((vy + 0.5) / 4.0 - 0.5, 0.0, double(irH - 1));
                    const int iy0 = int(fy), iy1 = std::min(iy0 + 1, irH - 1);
                    const unsigned wy = unsigned(std::lround((fy - iy0) * 256.0));
                    const uint16_t* vis = o.ch[0].peek(visY0 + vy);
                    const uint16_t* visB = scale > 1 ? o.ch[0].peek(visY0 + std::min(visH - 1, vy + 1)) : nullptr;
                    const uint16_t* ir0 = o.ch[3].peek(irY0 + iy0);
                    const uint16_t* ir1 = o.ch[3].peek(irY0 + iy1);
                    if (!ir0) ir0 = ir1;
                    if (!ir1) ir1 = ir0;
                    uint8_t* out = bits + qsizetype(y) * bpl;
                    for (int x = 0; x < fcW; ++x) {
                        unsigned v = 0;
                        if (vis) {
                            if (scale == 1) v = vis[x];
                            else {
                                const int xx = x * 2, x2 = std::min(xx + 1, visW - 1);
                                const unsigned s = unsigned(vis[xx]) + vis[x2] + (visB ? unsigned(visB[xx]) + visB[x2] : unsigned(vis[xx]) + vis[x2]);
                                v = s / 4;
                            }
                        }
                        unsigned ir = 0;
                        if (ir0) {
                            const unsigned a0 = ir0[ix0[size_t(x)]], a1 = ir0[ix1[size_t(x)]];
                            const unsigned b0 = ir1[ix0[size_t(x)]], b1 = ir1[ix1[size_t(x)]];
                            const unsigned w = wx[size_t(x)];
                            const unsigned top = (a0 * (256 - w) + a1 * w) >> 8, bot = (b0 * (256 - w) + b1 * w) >> 8;
                            ir = (top * (256 - wy) + bot * wy) >> 8;
                        }
                        const unsigned char* c = &kFalseColorLut[(toByte(ir) * 256u + toByte(v)) * 3u];
                        out[3 * x] = c[0]; out[3 * x + 1] = c[1]; out[3 * x + 2] = c[2];
                    }
                }
            };
            const int nt = std::clamp(int(std::thread::hardware_concurrency()) - 1, 1, 8);
            std::vector<std::thread> pool;
            const int step = (fcH + nt - 1) / nt;
            for (int t = 1; t < nt; ++t) pool.emplace_back(work, std::min(fcH, t * step), std::min(fcH, (t + 1) * step));
            work(0, std::min(fcH, step));
            for (auto& th : pool) th.join();
            const QString path = QDir(dir).filePath(QStringLiteral("GVAR_FalseColor.png"));
            if (fc.save(path, "PNG")) {
                paths << path;
                // display copy for the Image page (at most 4096 px on the long side, JPEG): the full PNG took seconds to
                // decode on the GUI thread
                if (fc.width() > 4096 || fc.height() > 4096) {
                    const QImage pv = fc.scaled(QSize(4096, 4096), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                    if (!pv.isNull() && QDir(dir).mkpath(QStringLiteral(".preview")))
                        pv.save(QDir(dir).filePath(QStringLiteral(".preview/GVAR_FalseColor.jpg")), "JPG", 90);
                }
                QJsonObject j;
                j[QStringLiteral("name")] = QStringLiteral("False colour (VIS x IR 10.7, SatDump table)");
                j[QStringLiteral("path")] = QStringLiteral("GVAR_FalseColor.png");
                j[QStringLiteral("width")] = fc.width();
                j[QStringLiteral("height")] = fc.height();
                j[QStringLiteral("vertical_stretch")] = 1.75;
                j[QStringLiteral("resolution")] = scale == 1 ? QStringLiteral("visible (1 km)") : QStringLiteral("half visible (2 km)");
                channelsJson.append(j);
            }
        }
    }

    // ---- channel images: original sampling, 16-bit (10-bit counts shifted into the high bits), cropped
    // to the transmitted frame, then stretched 1.75x vertically (the aspect correction) while being turned into the image.
    for (int c = 0; c < 5; ++c) {
        const bool vis = c == 0;
        const int w = vis ? visW : irW, h = vis ? visH : irH, y0 = vis ? visY0 : irY0;
        if (w <= 0 || h <= 0 || o.ch[c].firstRow() < 0) continue;
        QImage img = rasterToStretchedImage(o.ch[c], y0, h, w, kVertStretch100);   // stored with the aspect correction applied
        if (img.isNull()) {
            update.message = QStringLiteral("GOES GVAR: out of memory, %1 could not be produced").arg(QString::fromLatin1(kChannels[c].label));
            continue;
        }
        const QString path = QDir(dir).filePath(QString::fromLatin1(kChannels[c].file));
        if (img.save(path, "PNG")) {
            paths << path;
            QJsonObject j;
            j[QStringLiteral("name")] = QString::fromLatin1(kChannels[c].label);
            j[QStringLiteral("path")] = QString::fromLatin1(kChannels[c].file);
            j[QStringLiteral("wavelength_um")] = kChannels[c].micron;
            j[QStringLiteral("width")] = img.width();
            j[QStringLiteral("height")] = img.height();
            j[QStringLiteral("bits")] = 10;
            j[QStringLiteral("stored_bits")] = 16;
            j[QStringLiteral("vertical_stretch_applied")] = 1.75;
            j[QStringLiteral("source_lines")] = h;
            channelsJson.append(j);
        }
        o.ch[c].clear();
    }

    QJsonObject root;
    root[QStringLiteral("satellite")] = QStringLiteral("GOES-N/O/P (GVAR)");
    root[QStringLiteral("observation_utc")] = update.stamp;
    const QString frameKind = o.info.valid ? QString::fromUtf8(frameKindName(o.info.iwfpx, o.info.iefpx, o.info.infln, o.info.isfln))
                                           : QStringLiteral("unknown");
    root[QStringLiteral("frame_kind")] = frameKind;
    root[QStringLiteral("subsatellite_longitude_deg")] = o.info.subLon;
    root[QStringLiteral("scan_lines_received")] = o.lines;
    root[QStringLiteral("first_scan_counter")] = o.firstCounter;
    root[QStringLiteral("last_scan_counter")] = o.lastCounter;
    QJsonObject b0;
    b0[QStringLiteral("IWFPX")] = o.info.iwfpx; b0[QStringLiteral("IEFPX")] = o.info.iefpx;
    b0[QStringLiteral("INFLN")] = o.info.infln; b0[QStringLiteral("ISFLN")] = o.info.isfln;
    b0[QStringLiteral("INSLN")] = o.info.insln;
    root[QStringLiteral("block0")] = b0;
    QJsonObject dec;
    dec[QStringLiteral("blocks_used")] = qint64(o.blocksSeen);
    dec[QStringLiteral("blocks_soft_recovered")] = qint64(o.blocksRecovered);
    dec[QStringLiteral("blocks_truncated_skipped")] = qint64(o.blocksTruncated);
    dec[QStringLiteral("headers_length_corrected")] = qint64(o.headersCorrected);
    dec[QStringLiteral("ir_record_length_overridden")] = qint64(o.recOverridden);
    dec[QStringLiteral("ir_dead_tail_rows_skipped")] = qint64(o.deadTailRows);
    {
        QJsonArray ev;
        for (int i = 0; i < o.diagN; ++i) {
            QJsonArray row;
            for (int k = 0; k < 7; ++k) row.append(o.diag[i][k]);
            ev.append(row);
        }
        dec[QStringLiteral("ir_events_first_line_block_record_found_words_declared_docwords")] = ev;
    }
    {
        QJsonArray byId, lens;
        for (int i = 0; i <= 10; ++i) byId.append(o.truncatedById[i]);
        for (int i = 0; i < o.truncatedWordsN; ++i) lens.append(o.truncatedWords[i]);
        dec[QStringLiteral("truncated_by_block_id")] = byId;
        dec[QStringLiteral("truncated_words_first")] = lens;
    }
    dec[QStringLiteral("soft_pixel_recovery")] = o.softUsed;
    dec[QStringLiteral("visible_words_per_line")] = o.visWidthWords;
    dec[QStringLiteral("ir_words_per_record")] = o.irRecordWords;
    root[QStringLiteral("decode")] = dec;
    root[QStringLiteral("products")] = channelsJson;
    QFile jf(QDir(dir).filePath(QStringLiteral("GVAR_Observation.json")));
    if (jf.open(QIODevice::WriteOnly | QIODevice::Truncate)) jf.write(QJsonDocument(root).toJson(QJsonDocument::Indented));

    update.paths = paths;
    if (update.message.isEmpty())
        update.message = QStringLiteral("GOES GVAR · %1 · %2 · %3 lines · %4 images saved (false colour + 5 channels)").arg(update.stamp, frameKind).arg(o.lines).arg(paths.size());
    nadirstation::log::write(update.message);
    std::lock_guard<std::mutex> l(uMutex_);
    completed_.push_back(std::move(update));
}

}  // namespace nadirstation::gvar
