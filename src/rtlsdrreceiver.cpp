#include "rtlsdrreceiver.h"

#include <QLibrary>
#include <QCoreApplication>
#include <QDir>
#include <QProcessEnvironment>
#include <QFileInfo>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {
struct rtlsdr_dev;
using Device = rtlsdr_dev;
using GetDeviceCount = std::uint32_t (*)();
using Open = int (*)(Device**, std::uint32_t);
using Close = int (*)(Device*);
using SetU32 = int (*)(Device*, std::uint32_t);
using GetU32 = std::uint32_t (*)(Device*);
using SetGainMode = int (*)(Device*, int);
using SetGain = int (*)(Device*, int);
using GetTunerGains = int (*)(Device*, int*);
using SetBiasTee = int (*)(Device*, int);
using ReadAsyncCallback = void (*)(unsigned char*, std::uint32_t, void*);
using ReadAsync = int (*)(Device*, ReadAsyncCallback, void*, std::uint32_t, std::uint32_t);
using CancelAsync = int (*)(Device*);
using ResetBuffer = int (*)(Device*);
}

RtlSdrReceiver::RtlSdrReceiver(QObject* parent) : QObject(parent) {}
RtlSdrReceiver::~RtlSdrReceiver() { stop(); }

void RtlSdrReceiver::setParameters(double frequencyHz, double sampleRate,
                                   double bandwidthHz, double gainDb,
                                   unsigned deviceIndex, bool autoBiasTee) {
    frequencyHz_ = frequencyHz;
    sampleRate_ = sampleRate;
    bandwidthHz_ = bandwidthHz;
    gainDb_ = gainDb;
    deviceIndex_ = deviceIndex;
    autoBiasTee_ = autoBiasTee;
}

void RtlSdrReceiver::setSpectrumSettings(std::size_t fftSize, nadirstation::dsp::SpectrumWindow window) {
    fftSize_.store(fftSize);
    fftWindow_.store(static_cast<int>(window));
}

bool RtlSdrReceiver::start() {
    if (running_.exchange(true)) return true;
    if (worker_.joinable()) worker_.join();
    worker_ = std::thread(&RtlSdrReceiver::run, this);
    return true;
}

void RtlSdrReceiver::stop() {
    running_.store(false);
    {
        std::lock_guard<std::mutex> lock(deviceMutex_);
        if (activeDevice_ && cancelAsyncFunction_) {
            const auto cancel = reinterpret_cast<CancelAsync>(cancelAsyncFunction_);
            cancel(static_cast<Device*>(activeDevice_));
        }
    }
    if (worker_.joinable()) worker_.join();
}

void RtlSdrReceiver::cancelFromStream() {
    std::lock_guard<std::mutex> lock(deviceMutex_);
    if (activeDevice_ && cancelAsyncFunction_)
        reinterpret_cast<CancelAsync>(cancelAsyncFunction_)(static_cast<Device*>(activeDevice_));
}

// The driver gives no count of lost samples, but the stream has a known rate: when fewer samples
// have arrived than the clock says were due, the USB transfer dropped the rest. The comparison is
// cumulative from the start of the stream and looked at every five seconds. Blocks that are merely
// late (the host was busy, the driver queued them) are made up by the next look, so a shortfall is
// only reported once it has been there on two looks in a row.
void RtlSdrReceiver::accountSamples(std::size_t sampleCount, double actualRate) {
    const auto now = std::chrono::steady_clock::now();
    if (!rateWindowOpen_) {
        rateWindowOpen_ = true;
        rateStreamStart_ = rateLastCheck_ = now;
        rateTotalSamples_ = 0;
        rateConfirmedLost_ = 0.0;
        ratePendingBefore_ = 0.0;
        return;   // the first block only starts the clock: it was filled before this instant
    }
    rateTotalSamples_ += sampleCount;
    const double sinceCheck = std::chrono::duration<double>(now - rateLastCheck_).count();
    if (sinceCheck < 5.0 || actualRate <= 0.0) return;
    rateLastCheck_ = now;
    const double due = actualRate * std::chrono::duration<double>(now - rateStreamStart_).count();
    const double pending = due - static_cast<double>(rateTotalSamples_) - rateConfirmedLost_;
    const double threshold = 0.01 * actualRate * sinceCheck;
    if (pending > threshold && ratePendingBefore_ > threshold) {
        const double lost = std::min(pending, ratePendingBefore_);
        rateConfirmedLost_ += lost;
        droppedSamples_.fetch_add(static_cast<std::uint64_t>(lost));
        emit statusMessage(QStringLiteral("RTL-SDR samples lost: USB delivered about %1 fewer samples than expected (%2 ms of signal), %3 in total. Use a USB port directly on the motherboard, lower the sample rate, or close programs that load the CPU.")
                               .arg(static_cast<qulonglong>(lost))
                               .arg(1000.0 * lost / actualRate, 0, 'f', 0)
                               .arg(static_cast<qulonglong>(droppedSamples_.load())));
        ratePendingBefore_ = pending - lost;
    } else {
        ratePendingBefore_ = pending;
    }
}

void RtlSdrReceiver::consume(const unsigned char* bytes, std::uint32_t length, double actualRate,
                             std::deque<std::pair<std::chrono::steady_clock::time_point, float>>& history,
                             float& peak, std::chrono::steady_clock::time_point& lastMetric) {
    if (!running_.load() || !bytes || length < 2) return;
    const std::size_t sampleCount = static_cast<std::size_t>(length) / 2;
    accountSamples(sampleCount, actualRate);
    auto samples = std::make_shared<std::vector<std::complex<float>>>(sampleCount);
    for (std::size_t i = 0; i < sampleCount; ++i) {
        (*samples)[i] = {static_cast<float>(bytes[2 * i]) / 127.5f - 1.0f,
                         static_cast<float>(bytes[2 * i + 1]) / 127.5f - 1.0f};
    }
    // Waterfall rows at a fixed cadence in stream time; the SNR metrics stay at 10 per second.
    float latestSnr = 0.0f;
    bool haveSpectrum = false;
    spectrumTap_.feed(*samples, actualRate, fftSize_.load(),
                      static_cast<nadirstation::dsp::SpectrumWindow>(fftWindow_.load()),
                      [&](const nadirstation::dsp::SpectrumResult& sp) {
                          emit spectrumReady(sp.db, actualRate);
                          latestSnr = sp.snrDb;
                          haveSpectrum = true;
                      });
    const auto now = std::chrono::steady_clock::now();
    if (haveSpectrum && now - lastMetric >= std::chrono::milliseconds(100)) {
        peak = std::max(peak, latestSnr);
        history.emplace_back(now, latestSnr);
        while (!history.empty() && now - history.front().first > std::chrono::seconds(1)) history.pop_front();
        double sum = 0.0;
        for (const auto& v : history) sum += v.second;
        const float average = history.empty() ? 0.0f : static_cast<float>(sum / static_cast<double>(history.size()));
        emit metricsReady(latestSnr, peak, average, actualRate);
        lastMetric = now;
    }
    emit samplesReady(samples);
}

void RtlSdrReceiver::run() {
    QLibrary library;
    QStringList candidates;
    const QString configuredDll = QProcessEnvironment::systemEnvironment().value(QStringLiteral("NADIRSTATION_RTLSDR_DLL"));
    if (!configuredDll.isEmpty()) candidates.push_back(configuredDll);
    const QString appDir = QCoreApplication::applicationDirPath();
    candidates << QDir(appDir).filePath(QStringLiteral("rtlsdr.dll"))
               << QDir(appDir).filePath(QStringLiteral("librtlsdr.dll"));
    const QString programFiles = qEnvironmentVariable("ProgramFiles");
    if (!programFiles.isEmpty()) {
        candidates << QDir(programFiles).filePath(QStringLiteral("PothosSDR/bin/rtlsdr.dll"))
                   << QDir(programFiles).filePath(QStringLiteral("SDRangel/rtlsdr.dll"))
                   << QDir(programFiles).filePath(QStringLiteral("GNURadio-3.8/bin/rtlsdr.dll"));
    }
    candidates << QStringLiteral("rtlsdr") << QStringLiteral("librtlsdr");
    for (const QString& candidate : candidates) {
#ifdef Q_OS_WIN
        if (QFileInfo(candidate).isAbsolute()) {
            const std::wstring dllDirectory = QFileInfo(candidate).absolutePath().toStdWString();
            SetDllDirectoryW(dllDirectory.c_str());
        }
#endif
        library.setFileName(candidate);
        if (library.load()) break;
    }
    if (!library.isLoaded()) {
        emit statusMessage(QStringLiteral("RTL-SDR driver not found. Put a compatible rtlsdr.dll and its dependent DLLs in the program folder."));
        emit deviceConnected(false);
        running_.store(false);
        emit finished();
        return;
    }
    emit statusMessage(QStringLiteral("RTL-SDR driver loaded: %1").arg(library.fileName()));

    const auto getCount = reinterpret_cast<GetDeviceCount>(library.resolve("rtlsdr_get_device_count"));
    const auto open = reinterpret_cast<Open>(library.resolve("rtlsdr_open"));
    const auto close = reinterpret_cast<Close>(library.resolve("rtlsdr_close"));
    const auto setFrequency = reinterpret_cast<SetU32>(library.resolve("rtlsdr_set_center_freq"));
    const auto setRate = reinterpret_cast<SetU32>(library.resolve("rtlsdr_set_sample_rate"));
    const auto getRate = reinterpret_cast<GetU32>(library.resolve("rtlsdr_get_sample_rate"));
    const auto setBandwidth = reinterpret_cast<SetU32>(library.resolve("rtlsdr_set_tuner_bandwidth"));
    const auto setGainMode = reinterpret_cast<SetGainMode>(library.resolve("rtlsdr_set_tuner_gain_mode"));
    const auto setGain = reinterpret_cast<SetGain>(library.resolve("rtlsdr_set_tuner_gain"));
    const auto getTunerGains = reinterpret_cast<GetTunerGains>(library.resolve("rtlsdr_get_tuner_gains"));
    const auto setBiasTee = reinterpret_cast<SetBiasTee>(library.resolve("rtlsdr_set_bias_tee"));
    const auto readAsync = reinterpret_cast<ReadAsync>(library.resolve("rtlsdr_read_async"));
    const auto cancelAsync = reinterpret_cast<CancelAsync>(library.resolve("rtlsdr_cancel_async"));
    const auto resetBuffer = reinterpret_cast<ResetBuffer>(library.resolve("rtlsdr_reset_buffer"));
    if (!getCount || !open || !close || !setFrequency || !setRate || !getRate ||
        !setGainMode || !setGain || !readAsync || !cancelAsync) {
        emit statusMessage(QStringLiteral("The RTL-SDR driver lacks the receive API; use a compatible librtlsdr."));
        emit deviceConnected(false);
        running_.store(false);
        emit finished();
        return;
    }

    const std::uint32_t count = getCount();
    if (deviceIndex_ >= count) {
        emit statusMessage(QStringLiteral("RTL-SDR device index %1 is out of range; %2 device(s) detected.")
                               .arg(deviceIndex_).arg(count));
        emit deviceConnected(false);
        running_.store(false);
        emit finished();
        return;
    }

    Device* device = nullptr;
    int result = open(&device, deviceIndex_);
    if (result < 0 || !device) {
        emit statusMessage(QStringLiteral("Cannot open RTL-SDR device %1 (rc=%2); check the USB driver and whether another program is using it.")
                               .arg(deviceIndex_).arg(result));
        emit deviceConnected(false);
        running_.store(false);
        emit finished();
        return;
    }
    bool biasEnabled = false;
    auto fail = [this](const QString& what, int rc) {
        emit statusMessage(QStringLiteral("RTL-SDR setup failed: %1 (rc=%2)").arg(what).arg(rc));
    };
    auto setU32 = [&](SetU32 fn, std::uint32_t value, const QString& what) {
        const int rc = fn(device, value);
        if (rc < 0) fail(what, rc);
        return rc >= 0;
    };
    if (autoBiasTee_) {
        if (!setBiasTee) {
            emit statusMessage(QStringLiteral("The driver does not support Bias-T control; receiving not started. Use a driver that provides rtlsdr_set_bias_tee."));
            close(device);
            emit deviceConnected(false);
            running_.store(false);
            emit finished();
            return;
        }
        result = setBiasTee(device, 1);
        if (result < 0) {
            fail(QStringLiteral("Bias-T enable"), result);
            close(device);
            emit deviceConnected(false);
            running_.store(false);
            emit finished();
            return;
        }
        biasEnabled = true;
    }

    bool configured = setU32(setFrequency, static_cast<std::uint32_t>(std::llround(frequencyHz_)), QStringLiteral("centre frequency")) &&
        setU32(setRate, static_cast<std::uint32_t>(std::llround(sampleRate_)), QStringLiteral("sample rate")) &&
        (!setBandwidth || setU32(setBandwidth, static_cast<std::uint32_t>(std::llround(bandwidthHz_)), QStringLiteral("bandwidth")));
    if (configured) {
        result = setGainMode(device, 1);
        if (result < 0) { fail(QStringLiteral("manual gain mode"), result); configured = false; }
    }
    if (configured && getTunerGains) {
        const int gainCount = getTunerGains(device, nullptr);
        if (gainCount > 0) {
            std::vector<int> supported(static_cast<std::size_t>(gainCount));
            const int actualCount = getTunerGains(device, supported.data());
            if (actualCount > 0) {
                supported.resize(static_cast<std::size_t>(actualCount));
                const int requested = static_cast<int>(std::llround(gainDb_ * 10.0));
                const int selected = *std::min_element(supported.begin(), supported.end(), [requested](int a, int b) {
                    return std::abs(a - requested) < std::abs(b - requested);
                });
                result = setGain(device, selected);
            } else {
                result = setGain(device, static_cast<int>(std::llround(gainDb_ * 10.0)));
            }
        } else {
            result = setGain(device, static_cast<int>(std::llround(gainDb_ * 10.0)));
        }
        if (result < 0) { fail(QStringLiteral("manual gain"), result); configured = false; }
    } else if (configured) {
        result = setGain(device, static_cast<int>(std::llround(gainDb_ * 10.0)));
        if (result < 0) { fail(QStringLiteral("manual gain"), result); configured = false; }
    }
    if (!configured) {
        if (biasEnabled) setBiasTee(device, 0);
        close(device);
        emit deviceConnected(false);
        running_.store(false);
        emit finished();
        return;
    }

    const double actualRate = static_cast<double>(getRate(device));
    emit statusMessage(QStringLiteral("RTL-SDR connected: device %1 · %2 MS/s · %3 MHz · Bias-T %4")
                           .arg(deviceIndex_).arg(actualRate / 1e6, 0, 'f', 3)
                           .arg(bandwidthHz_ / 1e6, 0, 'f', 3)
                           .arg(biasEnabled ? QStringLiteral("on") : QStringLiteral("off")));
    emit deviceConnected(true);

    spectrumTap_.reset();
    rateWindowOpen_ = false;
    droppedSamples_.store(0);
    {
        std::lock_guard<std::mutex> lock(deviceMutex_);
        activeDevice_ = device;
        cancelAsyncFunction_ = reinterpret_cast<void*>(cancelAsync);
    }
    struct StreamState {
        RtlSdrReceiver* self;
        double sampleRate;
        std::atomic<bool>* hasSamples;
        std::deque<std::pair<std::chrono::steady_clock::time_point, float>> history;
        float peak = -120.0f;
        std::chrono::steady_clock::time_point lastMetric = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    };
    std::atomic<bool> hasSamples{false};
    StreamState stream{this, actualRate, &hasSamples, {}, -120.0f,
                       std::chrono::steady_clock::now() - std::chrono::seconds(1)};
    if (resetBuffer) {
        const int resetResult = resetBuffer(device);
        if (resetResult < 0) emit statusMessage(QStringLiteral("RTL-SDR USB buffer reset failed (rc=%1); receive may fail.")
                                                .arg(resetResult));
    }
    std::atomic<bool> streamReturned{false};
    std::thread watchdog([&] {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (running_.load() && !streamReturned.load() && !hasSamples.load() &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (running_.load() && !streamReturned.load() && !hasSamples.load()) {
            emit statusMessage(QStringLiteral("RTL-SDR opened, but no IQ data arrived within 5 s; check the USB driver, the antenna and whether another program is using the device."));
            std::lock_guard<std::mutex> lock(deviceMutex_);
            if (activeDevice_ && cancelAsyncFunction_)
                reinterpret_cast<CancelAsync>(cancelAsyncFunction_)(static_cast<Device*>(activeDevice_));
        }
    });
    const int streamResult = running_.load()
        ? readAsync(device, [](unsigned char* bytes, std::uint32_t length, void* context) {
              auto* state = static_cast<StreamState*>(context);
              if (state && state->self) {
                  if (!state->self->running_.load()) {
                      // stop() can run after the device was published but before the driver's stream
                      // was live; its cancel is then ignored by the driver (the stream is not active
                      // yet), the stream starts anyway, and stop() waits for this thread for ever --
                      // with the GUI, which is the caller. Cancelling again from inside the stream
                      // always lands.
                      state->self->cancelFromStream();
                      return;
                  }
                  if (bytes && length >= 2) state->hasSamples->store(true);
                  state->self->consume(bytes, length, state->sampleRate, state->history,
                                       state->peak, state->lastMetric);
              }
          }, &stream, 0, 0)
        : 0;
    streamReturned.store(true);
    if (watchdog.joinable()) watchdog.join();
    {
        std::lock_guard<std::mutex> lock(deviceMutex_);
        activeDevice_ = nullptr;
        cancelAsyncFunction_ = nullptr;
    }
    if (streamResult < 0 && running_.load())
        emit statusMessage(QStringLiteral("RTL-SDR IQ read failed (rc=%1).").arg(streamResult));
    if (biasEnabled) {
        const int rc = setBiasTee(device, 0);
        emit statusMessage(rc < 0 ? QStringLiteral("RTL-SDR stopped, but switching Bias-T off failed (rc=%1).").arg(rc)
                                  : QStringLiteral("RTL-SDR stopped, Bias-T off."));
    }
    close(device);
    emit deviceConnected(false);
    running_.store(false);
    emit finished();
}
