#pragma once

#include <QObject>
#include <QString>
#include <atomic>
#include <complex>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <deque>
#include "dsp.h"
#include "spectrumtap.h"

class QLibrary;

class RtlSdrReceiver : public QObject {
    Q_OBJECT
public:
    explicit RtlSdrReceiver(QObject* parent = nullptr);
    ~RtlSdrReceiver() override;

    void setParameters(double frequencyHz, double sampleRate, double bandwidthHz,
                       double gainDb, unsigned deviceIndex, bool autoBiasTee);
    void setSpectrumSettings(std::size_t fftSize, nadirstation::dsp::SpectrumWindow window);
    bool start();
    void stop();
    bool isRunning() const { return running_.load(); }
    // As last given to setParameters (GUI thread).
    double sampleRate() const { return sampleRate_; }
    double frequencyHz() const { return frequencyHz_; }
    // Samples the USB transfer failed to deliver since the last start (estimated from the stream rate).
    std::uint64_t droppedSamples() const { return droppedSamples_.load(); }

signals:
    void samplesReady(const std::shared_ptr<std::vector<std::complex<float>>>& samples);
    void metricsReady(float snr, float peakSnr, float avgSnr1s, double sampleRate);
    void spectrumReady(const std::vector<float>& db, double sampleRate);
    void statusMessage(const QString& text);
    void deviceConnected(bool connected);
    void finished();

private:
    void run();
    void consume(const unsigned char* bytes, std::uint32_t length, double actualRate,
                 std::deque<std::pair<std::chrono::steady_clock::time_point, float>>& history,
                 float& peak, std::chrono::steady_clock::time_point& lastMetric);
    void cancelFromStream();
    void accountSamples(std::size_t sampleCount, double actualRate);
    // Rate check, touched only by the stream callback.
    bool rateWindowOpen_ = false;
    std::chrono::steady_clock::time_point rateStreamStart_{}, rateLastCheck_{};
    std::uint64_t rateTotalSamples_ = 0;
    double rateConfirmedLost_ = 0.0, ratePendingBefore_ = 0.0;
    std::atomic<std::uint64_t> droppedSamples_{0};
    mutable std::mutex deviceMutex_;
    void* activeDevice_ = nullptr;
    void* cancelAsyncFunction_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<std::size_t> fftSize_{2048};
    std::atomic<int> fftWindow_{static_cast<int>(nadirstation::dsp::SpectrumWindow::Hann)};
    double frequencyHz_ = 1687500000.0;
    double sampleRate_ = 2400000.0;
    double bandwidthHz_ = 2000000.0;
    double gainDb_ = 35.0;
    unsigned deviceIndex_ = 0;
    bool autoBiasTee_ = true;
    std::thread worker_;
    nadirstation::dsp::SpectrumTap spectrumTap_;   // used only on the worker thread
};
