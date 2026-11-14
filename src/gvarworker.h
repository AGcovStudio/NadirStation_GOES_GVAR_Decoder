// gvarworker.h - runs the GVAR receive chain on its own thread, fed either from a recording or from the RTL-SDR,
// and reports to the window. One engine, two sources: the signal path after the samples are the same.
#pragma once
#include "gvarengine.h"

#include <QImage>
#include <QMetaType>
#include <QObject>
#include <QString>
#include <QStringList>
#include <atomic>
#include <complex>
#include <memory>
#include <vector>

struct GvarStats {
    bool locked = false;
    float lockQuality = 0.f;       // carrier measure, 0..1
    float carrierHz = 0.f;
    double snrDb = 0.0;
    qulonglong syncs = 0, blocksGood = 0, blocksBad = 0, lines = 0, softBlocks = 0;
    int queuedBlocks = 0;
    qulonglong droppedIqBlocks = 0;   // live only: IQ blocks thrown away because the decoder fell behind
};
Q_DECLARE_METATYPE(GvarStats)
Q_DECLARE_METATYPE(std::vector<std::complex<float>>)
Q_DECLARE_METATYPE(std::vector<float>)

class GvarWorker : public QObject {
    Q_OBJECT
public:
    using Samples = std::shared_ptr<std::vector<std::complex<float>>>;
    static constexpr double kSymbolRate = 2111360.0;                  // GVAR, symbols per second
    static constexpr double kMinSampleRate = 1.3 * kSymbolRate;       // the demodulator's lower limit

    explicit GvarWorker(QObject* parent = nullptr);
    ~GvarWorker() override;

    // Thread-safe.
    void requestStop() { stop_.store(true); }
    // Live source: called from the receiver's thread for every block of samples.
    void pushSamples(const Samples& samples);

public slots:
    void runFile(const QString& inputPath, double manualRateHz, bool preferManual, const QString& outputDir);
    bool beginLive(double sampleRate, const QString& outputDir);   // false: finished(false, ...) has been sent
    void endLive();

signals:
    void inputInfo(const QString& description, double sampleRate);
    void logLine(const QString& text);
    void progress(int permille);
    void statsChanged(const GvarStats& stats);
    void constellation(const std::vector<std::complex<float>>& points);
    void spectrum(const std::vector<float>& db, double sampleRate);
    void livePreview(const QImage& image, const QString& stamp);
    void observationDone(const QString& stamp, const QStringList& paths);
    void telemetry(const QString& spacecraft, const QString& time);
    void finished(bool ok, const QString& summary);

private:
    bool prepare(double sampleRate, const QString& outputDir, QString& error);
    void processBlock(const std::vector<std::complex<float>>& iq, double sampleRate);
    void tick(bool force);
    void publishResults();
    void publishTelemetry(bool force);
    QString finish(bool stopped);

    std::unique_ptr<nadirstation::gvar::Engine> engine_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> live_{false};
    std::atomic<int> pending_{0};
    std::atomic<qulonglong> dropped_{0};

    double sampleRate_ = 0.0;
    std::vector<std::complex<float>> lastSymbols_;
    qint64 lastTickMs_ = 0, lastSpectrumMs_ = 0, lastTelemetryMs_ = 0, lastLogMs_ = 0, lastSyncMs_ = 0;
    uint64_t lastSyncs_ = 0;
    int observations_ = 0;
    bool fileMode_ = false;
    qulonglong reportedDropped_ = 0;
};
