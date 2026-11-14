#include "gvarworker.h"

#include "iqreader.h"
#include "qtpath.h"
#include "runtime_logger.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QMetaObject>
#include <QThread>
#include <algorithm>
#include <chrono>

namespace {
constexpr int kMaxPendingBlocks = 100;       // live: about 4 s of samples may wait for the decoder
constexpr std::size_t kSpectrumBins = 2048;

qint64 nowMs() { return QDateTime::currentMSecsSinceEpoch(); }
}  // namespace

GvarWorker::GvarWorker(QObject* parent) : QObject(parent) {}

GvarWorker::~GvarWorker() = default;

bool GvarWorker::prepare(double sampleRate, const QString& outputDir, QString& error) {
    if (sampleRate < kMinSampleRate) {
        error = QStringLiteral("Sample rate %1 MS/s is too low: GOES GVAR (2.111 Msym/s) needs at least %2 MS/s; 2.88 MS/s or more is recommended.")
                    .arg(sampleRate / 1e6, 0, 'f', 3).arg(kMinSampleRate / 1e6, 0, 'f', 2);
        return false;
    }
    if (!QDir().mkpath(outputDir)) {
        error = QStringLiteral("Cannot create the output folder: %1").arg(outputDir);
        return false;
    }
    if (!engine_) engine_ = std::make_unique<nadirstation::gvar::Engine>();
    engine_->setOutputRoot(outputDir);
    // RRC roll-off 0.5 and a loop bandwidth of 0.02 (the engine narrows it for the continuous GVAR carrier).
    if (!engine_->configure(sampleRate, kSymbolRate, 0.5f, 0.02f)) {
        error = QStringLiteral("The demodulator cannot work at %1 MS/s.").arg(sampleRate / 1e6, 0, 'f', 3);
        return false;
    }
    engine_->setSoftRecovery(true);
    engine_->reset();
    sampleRate_ = sampleRate;
    lastSymbols_.clear();
    lastTickMs_ = lastSpectrumMs_ = lastTelemetryMs_ = lastLogMs_ = lastSyncMs_ = 0;
    lastSyncs_ = 0;
    observations_ = 0;
    pending_ = 0;
    dropped_ = 0;
    reportedDropped_ = 0;
    return true;
}

void GvarWorker::processBlock(const std::vector<std::complex<float>>& iq, double sampleRate) {
    if (iq.empty() || !engine_) return;
    const auto d = engine_->process(iq);
    if (!d.symbols.empty()) {
        lastSymbols_.insert(lastSymbols_.end(), d.symbols.begin(), d.symbols.end());
        if (lastSymbols_.size() > 2500) lastSymbols_.erase(lastSymbols_.begin(), lastSymbols_.end() - 2500);
    }
    const qint64 now = nowMs();
    if (now - lastSpectrumMs_ >= 80 && iq.size() >= kSpectrumBins) {
        lastSpectrumMs_ = now;
        const auto sp = nadirstation::dsp::spectrum(iq, static_cast<float>(sampleRate), kSpectrumBins);
        if (!sp.db.empty()) emit spectrum(sp.db, sampleRate);
    }
    tick(false);
}

void GvarWorker::publishResults() {
    auto live = engine_->takeLive();
    if (!live.live.isNull()) emit livePreview(live.live, live.stamp);
    if (!live.message.isEmpty()) emit logLine(live.message);
    for (auto& done : engine_->takeCompleted()) {
        if (!done.message.isEmpty()) emit logLine(done.message);
        if (!done.paths.isEmpty()) {
            ++observations_;
            emit observationDone(done.stamp, done.paths);
        }
    }
}

void GvarWorker::publishTelemetry(bool force) {
    const qint64 now = nowMs();
    if (!force && lastTelemetryMs_ != 0 && now - lastTelemetryMs_ < 1000) return;
    lastTelemetryMs_ = now;
    nadirstation::gvar::Engine::Telemetry t;
    if (engine_->takeTelemetry(t) && t.window > 0) {
        const auto& defs = nadirstation::gvar::docFieldDefs();
        const int spc = nadirstation::gvar::docFieldIndex("SPCID");
        const int tm = nadirstation::gvar::docFieldIndex("TCURR");
        if (spc >= 0 && tm >= 0)
            emit telemetry(QString::fromStdString(nadirstation::gvar::formatField(defs[size_t(spc)], t.consensus.data())),
                           QString::fromStdString(nadirstation::gvar::formatField(defs[size_t(tm)], t.consensus.data())));
    }
    engine_->saveTelemetry(force);
}

void GvarWorker::tick(bool force) {
    const qint64 now = nowMs();
    if (!force && lastTickMs_ != 0 && now - lastTickMs_ < 150) return;
    lastTickMs_ = now;

    GvarStats s;
    s.syncs = engine_->syncs();
    s.blocksGood = engine_->blocksGood();
    s.blocksBad = engine_->blocksBad();
    s.lines = engine_->linesStored();
    s.softBlocks = engine_->softBlocks();
    s.queuedBlocks = engine_->queuedBlocks();
    s.snrDb = engine_->snrDb();
    s.lockQuality = std::clamp(engine_->lockMeasure(), 0.f, 1.f);
    s.carrierHz = engine_->carrierOffsetHz();
    s.droppedIqBlocks = dropped_.load();
    // Locked while block sync words keep arriving: one within the last 1.5 s. (A block is 0.04 .. 0.1 s, and the
    // carrier measure falls with the SNR while blocks still decode, so it only counts when clearly high.)
    if (s.syncs != lastSyncs_) { lastSyncs_ = s.syncs; lastSyncMs_ = now; }
    s.locked = (s.syncs > 0 && lastSyncMs_ != 0 && now - lastSyncMs_ <= 1500) || engine_->lockMeasure() > 0.5f;
    emit statsChanged(s);
    if (!lastSymbols_.empty()) emit constellation(lastSymbols_);
    publishResults();
    publishTelemetry(false);

    if (s.droppedIqBlocks > reportedDropped_ && now - lastLogMs_ >= 2000) {
        emit logLine(QStringLiteral("Decoding cannot keep up with the input; %1 IQ blocks dropped so far. Lower the sample rate or close programs that load the CPU.")
                         .arg(s.droppedIqBlocks));
        reportedDropped_ = s.droppedIqBlocks;
        lastLogMs_ = now;
    }
    if (lastLogMs_ == 0 || now - lastLogMs_ >= 5000) {
        lastLogMs_ = now;
        const QString m = QStringLiteral("sync %1 · header CRC ok %2 / bad %3 · lines %4 · soft blocks %5 · SNR %6 dB · carrier quality %7%")
                              .arg(s.syncs).arg(s.blocksGood).arg(s.blocksBad).arg(s.lines).arg(s.softBlocks)
                              .arg(s.snrDb, 0, 'f', 1).arg(s.lockQuality * 100.f, 0, 'f', 0);
        emit logLine(m);
        nadirstation::log::write(m);
    }
}

QString GvarWorker::finish(bool stopped) {
    emit logLine(QStringLiteral("Assembling and saving images (up to a minute)…"));
    engine_->finish();          // blocks until the assembler and the writer are empty
    publishResults();
    publishTelemetry(true);
    tick(true);
    QString summary = QStringLiteral("%1: sync %2 · header CRC ok %3 / bad %4 · lines %5 · %6 observation(s) written")
                          .arg(stopped ? QStringLiteral("Stopped") : QStringLiteral("Decoding finished"))
                          .arg(engine_->syncs()).arg(engine_->blocksGood()).arg(engine_->blocksBad())
                          .arg(engine_->linesStored()).arg(observations_);
    if (engine_->syncs() == 0)
        summary += QStringLiteral("\nNo GVAR sync word found: check the frequency (1685.7 MHz), sample rate, gain and antenna; for a recording check its centre frequency.");
    else if (observations_ == 0)
        summary += QStringLiteral("\nToo few scan lines were received to form an image (a full disk needs about 26 minutes of continuous reception).");
    nadirstation::log::write(summary);
    return summary;
}

void GvarWorker::runFile(const QString& inputPath, double manualRateHz, bool preferManual, const QString& outputDir) {
    stop_.store(false);
    live_.store(false);
    fileMode_ = true;

    gvarapp::IqReader reader;
    std::string error;
    if (!reader.open(toFsPath(inputPath), manualRateHz, error, preferManual)) {
        emit finished(false, QString::fromUtf8(error.c_str()));
        return;
    }
    const gvarapp::IqInfo info = reader.info();
    const double seconds = static_cast<double>(info.frames) / info.sampleRate;
    emit inputInfo(QStringLiteral("%1 · %2 MS/s (%3) · %4 s")
                       .arg(QString::fromUtf8(gvarapp::formatName(info.format)))
                       .arg(info.sampleRate / 1e6, 0, 'f', 3)
                       .arg(QString::fromUtf8(info.rateSource.c_str()))
                       .arg(seconds, 0, 'f', 1),
                   info.sampleRate);
    QString prepError;
    if (!prepare(info.sampleRate, outputDir, prepError)) {
        emit finished(false, prepError);
        return;
    }
    emit logLine(QStringLiteral("Decoding started: %1 MS/s, about %2 s of data.").arg(info.sampleRate / 1e6, 0, 'f', 3).arg(seconds, 0, 'f', 1));

    std::vector<std::complex<float>> block;
    constexpr std::size_t kBlockFrames = 65536;
    bool stopped = false;
    int lastPermille = -1;
    while (reader.read(block, kBlockFrames) > 0) {
        if (stop_.load()) { stopped = true; break; }
        // A file is read far faster than real time; let the image assembler keep up instead of overflowing its queue.
        for (int waits = 0; engine_->queuedBlocks() > 6 && waits < 5000 && !stop_.load(); ++waits) QThread::msleep(2);
        processBlock(block, info.sampleRate);
        const int permille = static_cast<int>(1000.0 * static_cast<double>(reader.position()) / static_cast<double>(info.frames));
        if (permille != lastPermille) { lastPermille = permille; emit progress(permille); }
    }
    const QString summary = finish(stopped);
    emit progress(stopped ? std::max(lastPermille, 0) : 1000);
    emit finished(true, summary);
}

bool GvarWorker::beginLive(double sampleRate, const QString& outputDir) {
    stop_.store(false);
    live_.store(false);
    fileMode_ = false;
    QString error;
    if (!prepare(sampleRate, outputDir, error)) {
        emit finished(false, error);
        return false;
    }
    live_.store(true);
    emit logLine(QStringLiteral("Live reception: %1 MS/s, waiting for the GVAR signal…").arg(sampleRate / 1e6, 0, 'f', 3));
    return true;
}

void GvarWorker::pushSamples(const Samples& samples) {
    if (!samples || samples->empty() || !live_.load()) return;
    if (pending_.load() > kMaxPendingBlocks) {
        ++dropped_;
        return;
    }
    ++pending_;
    QMetaObject::invokeMethod(this, [this, samples] {
        --pending_;
        if (live_.load()) processBlock(*samples, sampleRate_);
    }, Qt::QueuedConnection);
}

void GvarWorker::endLive() {
    if (!engine_ || !live_.exchange(false)) return;
    const QString summary = finish(true);
    emit finished(true, summary);
}
