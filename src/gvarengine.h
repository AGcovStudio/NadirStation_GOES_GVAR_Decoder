#pragma once
// GOES-N/O/P GVAR receive chain for the station: BPSK demodulator -> soft block deframer -> image
// assembler (with soft pixel recovery) -> channel PNGs, false colour and JSON.
//
// Threads: process() runs on the caller's DSP thread (demodulation and block sync); a worker thread
// assembles lines (that is where the soft pixel recovery takes its CPU), and a writer thread saves
// finished observations, so neither can stall the IQ path. The block queue is bounded: when the
// assembler falls behind, soft recovery is switched off for the blocks in flight (hard decisions
// cost nothing) instead of dropping lines.
#include "dsp.h"
#include "gvardecoder.h"
#include "gvardoc.h"
#include "gvarimage.h"
#include <QImage>
#include <QString>
#include <QStringList>
#include <array>
#include <atomic>
#include <condition_variable>
#include <complex>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace nadirstation::gvar {

class Engine {
public:
    struct Update {
        QString stamp;          // observation the live view belongs to
        QString message;
        QImage live;            // 8-bit visible quicklook of the lines received so far (memory only)
        QStringList paths;      // finished observation: its products, FalseColor first
        bool complete = false;
    };

    Engine();
    ~Engine();
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // Rates in Hz; callable while idle. Returns false when the sample rate cannot hold the signal.
    bool configure(double sampleRate, double symbolRate, float rrcAlpha, float loopBandwidth);
    void setOutputRoot(const QString& root);
    void setSoftRecovery(bool enabled) { softWanted_ = enabled; }
    void reset();                                       // start of a reception

    // DSP thread. Returns the demodulated symbols for the constellation view.
    dsp::DPSKResult process(const std::vector<std::complex<float>>& iq);

    // Called from any thread.
    Update takeLive();                                  // latest live view and message (consumes them)
    std::vector<Update> takeCompleted();                // finished observations since the last call
    void finish();                                      // end of reception: flush everything to disk (blocks)

    // Link statistics for the UI.
    float lockMeasure() const { return lock_.load(); }
    float carrierOffsetHz() const { return carrierHz_.load(); }
    double snrDb() const { return snr_.load(); }
    uint64_t blocksGood() const { return blocksGood_.load(); }
    uint64_t blocksBad() const { return blocksBad_.load(); }
    uint64_t syncs() const { return syncs_.load(); }
    uint64_t linesStored() const { return lines_.load(); }
    uint64_t softBlocks() const { return softBlocks_.load(); }
    int queuedBlocks() const;
    bool configured() const { return configured_; }
    double sampleRate() const { return sampleRate_; }

    // Telemetry: block 0 (documentation) and the other non-imager blocks, for the telemetry page and the files.
    struct TelemetryProduct {
        int blockId = 0, productId = 0, wordSize = 0, wordCount = 0;
        uint64_t count = 0;                 // blocks of this id / product received
        std::vector<uint8_t> bytes;         // the latest one
    };
    struct Telemetry {
        std::vector<uint8_t> block0;                        // latest block 0 as received
        std::array<uint8_t, kDocBytes> consensus{};         // field-by-field majority over the window
        std::vector<int> votes;                             // per documentation field: blocks agreeing with it
        int window = 0;                                     // blocks 0 in the consensus window
        uint64_t blocks0 = 0, parityOk = 0;
        int errBits = 0, comparedBits = 0;                  // latest block 0 against the consensus
        std::vector<TelemetryProduct> products;
    };
    bool takeTelemetry(Telemetry& out);                 // false when nothing new since the last call
    void saveTelemetry(bool force);                     // write the telemetry folder (at most every 5 s unless forced)

private:
    void assemblerLoop();
    void writerLoop();
    void onLine(const Observation& o, int counter);
    void pushLive(const QString& message);
    void writeObservation(const std::shared_ptr<Observation>& o);
    void queueObservation(std::shared_ptr<Observation> o);
    void collectTelemetry(const Block& b);

    dsp::PskDemod demod_;
    std::vector<std::complex<float>> pendingFront_;   // filtered samples of the previous block, for the loop stage
    std::future<void> deframeJob_;                      // block sync of the block before (pipeline stage 3)
    void handleSymbols(const std::vector<std::complex<float>>& symbols);
    Deframer deframer_;
    bool configured_ = false;
    double sampleRate_ = 0.0, symbolRate_ = 0.0;
    QString outputRoot_;
    std::atomic<bool> softWanted_{true};

    // block queue (DSP thread -> assembler)
    mutable std::mutex qMutex_;
    std::condition_variable qCv_, qIdleCv_;
    std::deque<Block> blocks_;
    bool qStop_ = false, qFinish_ = false, qBusy_ = false;
    std::thread assembler_;
    ImageAssembler assemblerCore_;

    // finished observations (assembler -> writer)
    std::mutex wMutex_;
    std::condition_variable wCv_, wIdleCv_;
    std::deque<std::shared_ptr<Observation>> toWrite_;
    bool wStop_ = false, wBusy_ = false;
    std::thread writer_;

    // results
    std::mutex uMutex_;
    Update live_;
    std::vector<Update> completed_;

    QImage preview_;           // 8-bit, one row per scan line (counter), 16 visible pixels wide per column
    int previewFirst_ = -1, previewLast_ = -1;
    QString previewStamp_;
    qint64 lastLiveMs_ = 0;

    double snrY1_ = 0.0, snrY2_ = 0.0;
    std::atomic<float> lock_{0.f}, carrierHz_{0.f};
    std::atomic<double> snr_{0.0};
    std::atomic<uint64_t> blocksGood_{0}, blocksBad_{0}, syncs_{0}, lines_{0}, softBlocks_{0};
    std::atomic<uint64_t> softSkipped_{0};
    uint64_t lastBlocks_ = 0;

    // telemetry (block sync stage -> UI / files)
    std::mutex tMutex_;
    DocConsensus doc_;
    std::vector<uint8_t> block0Raw_;
    std::vector<TelemetryProduct> products_;
    std::vector<std::string> csvPending_;
    uint64_t blocks0_ = 0, parityOk_ = 0, telemetrySeen_ = 0, telemetryTaken_ = 0, savedBlocks0_ = 0;
    int lastErrBits_ = 0, lastCompared_ = 0;
    QString telemetryDir_;
    qint64 lastTelemetrySaveMs_ = 0;
};

}  // namespace nadirstation::gvar
