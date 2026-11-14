#include "runtime_logger.h"
#include <QCoreApplication>
#include <QDir>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QThread>
#include <QMutex>
#include <QTextStream>
#include <QDebug>
#include <QRegularExpression>
#include <QRecursiveMutex>
#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <thread>

#ifdef Q_OS_WIN
#include <windows.h>
#include <dbghelp.h>
#endif

namespace nadirstation::log {
namespace {
QRecursiveMutex g_mutex;   // recursive: a Qt warning raised while a line is written logs through the same lock
QString g_path;
QFile g_file;
QTextStream g_stream;
quint64 g_writeCount = 0;
constexpr qint64 MAX_LOG_BYTES = 30ll * 1024ll * 1024ll;

void reopenTruncatedUnlocked() {
    g_stream.flush();
    g_file.close();
    g_file.setFileName(g_path);
    if (!g_file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
    g_stream.setDevice(&g_file);
    g_stream << "===== NADIR STATION runtime log rotated at 30 MiB =====\n";
    g_stream << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs) << " LOG CLEARED\n";
    g_stream.flush();
}

void appendLineUnlocked(const QString& s) {
    if (g_path.isEmpty()) return;
    if (!g_file.isOpen()) {
        g_file.setFileName(g_path);
        if (!g_file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) return;
        g_stream.setDevice(&g_file);
    }
    const QString line = QStringLiteral("%1 [tid=%2] %3\n")
        .arg(QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs),
             QString::number(reinterpret_cast<quintptr>(QThread::currentThreadId()), 16), s);
    if (g_file.size() + line.toUtf8().size() > MAX_LOG_BYTES) reopenTruncatedUnlocked();
    if (!g_file.isOpen()) return;
    g_stream << line;
    // Keep the hot path buffered. Flush periodically instead of opening/flushing
    // a new QFile for every message; the previous behavior could visibly stutter
    // during continuous S-VISSR reception.
    // Flushed as written. Buffering every thirty-second line loses the end of any session shorter
    // than that, and the end is the part that is read: a live session that acquired a carrier wrote
    // ten lines in total, so nothing after startup reached the disk, and the log looked like a
    // receiver that never locked. The rate here is a few lines a second, and a flush on a file that
    // small costs nothing next to not being able to say what happened.
    g_stream.flush();
}

#ifdef Q_OS_WIN
LONG WINAPI unhandledExceptionFilter(EXCEPTION_POINTERS* ep) {
    const QString base = QCoreApplication::applicationDirPath();
    const QString dir = QDir(base).filePath("NadirStation_DATA");
    QDir().mkpath(dir);
    const QString logFile = QDir(dir).filePath("crash.log");
    const QString dumpFile = QDir(dir).filePath(QStringLiteral("crash_%1.dmp")
        .arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz")));

    const QString msg = QStringLiteral("UNHANDLED EXCEPTION code=0x%1 address=0x%2")
        .arg(ep && ep->ExceptionRecord ? ep->ExceptionRecord->ExceptionCode : 0, 8, 16, QLatin1Char('0'))
        .arg(ep && ep->ExceptionRecord ? reinterpret_cast<quintptr>(ep->ExceptionRecord->ExceptionAddress) : 0, 0, 16);
    QFile f(logFile);
    if (f.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        QTextStream ts(&f);
        ts << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs) << " " << msg << "\n";
        ts.flush();
    }

    HANDLE hFile = CreateFileW(reinterpret_cast<LPCWSTR>(dumpFile.utf16()), GENERIC_WRITE, 0, nullptr,
                                CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile != INVALID_HANDLE_VALUE) {
        MINIDUMP_EXCEPTION_INFORMATION mei{};
        mei.ThreadId = GetCurrentThreadId();
        mei.ExceptionPointers = ep;
        mei.ClientPointers = FALSE;
        MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, MiniDumpWithIndirectlyReferencedMemory,
                          ep ? &mei : nullptr, nullptr, nullptr);
        CloseHandle(hFile);
    }
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif
}

void init() {
    QMutexLocker locker(&g_mutex);
    g_path = QDir(QCoreApplication::applicationDirPath()).filePath("NadirStation_DATA/runtime.log");
    QDir().mkpath(QFileInfo(g_path).absolutePath());
    g_file.close();
    g_file.setFileName(g_path);
    if (QFileInfo::exists(g_path) && QFileInfo(g_path).size() >= MAX_LOG_BYTES)
        QFile::remove(g_path);
    if (g_file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        g_stream.setDevice(&g_file);
        g_stream << "\n===== NADIR STATION runtime log =====\n";
        g_stream << QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs) << " START\n";
        g_stream.flush();
        g_writeCount = 0;
    }
#ifdef Q_OS_WIN
    SetUnhandledExceptionFilter(unhandledExceptionFilter);
#endif
}

void write(const QString& message) {
    // Built once: constructing (and compiling) the pattern for every line was most of the cost of a
    // log call, and some callers log from the capture and decoder threads.
    static const QRegularExpression referenceName(QStringLiteral("sat.?dump"), QRegularExpression::CaseInsensitiveOption);
    QString safeMessage = message;
    safeMessage.replace(referenceName, QStringLiteral("reference-compatible"));
    QMutexLocker locker(&g_mutex);
    appendLineUnlocked(safeMessage);
    qInfo().noquote() << safeMessage;
}

void flush() {
    QMutexLocker locker(&g_mutex);
    if (g_file.isOpen()) g_stream.flush();
}

QString path() {
    QMutexLocker locker(&g_mutex);
    return g_path;
}
namespace {
QtMessageHandler g_previousHandler = nullptr;
std::atomic<int> g_qtWarnings{0};

void qtMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    // write() itself forwards every line with qInfo(), so only warnings and worse are taken here.
    if (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) {
        const int n = ++g_qtWarnings;
        if (type != QtWarningMsg || n <= 500) {
            const char* kind = type == QtWarningMsg ? "Qt warning" : (type == QtCriticalMsg ? "Qt critical" : "Qt FATAL");
            QString where;
            if (context.file) where = QStringLiteral(" (%1:%2)").arg(QString::fromUtf8(context.file)).arg(context.line);
            QMutexLocker locker(&g_mutex);
            appendLineUnlocked(QStringLiteral("%1: %2%3").arg(QString::fromLatin1(kind), msg, where));
        }
        if (type == QtFatalMsg) {
            writeDump(QStringLiteral("Qt fatal: %1").arg(msg));
        }
    }
    if (g_previousHandler) g_previousHandler(type, context, msg);
}

void onTerminate() {
    QString what = QStringLiteral("no active exception");
    if (const std::exception_ptr ex = std::current_exception()) {
        try { std::rethrow_exception(ex); }
        catch (const std::exception& e) { what = QStringLiteral("exception: %1").arg(QString::fromLocal8Bit(e.what())); }
        catch (...) { what = QStringLiteral("non-standard exception"); }
    }
    {
        QMutexLocker locker(&g_mutex);
        appendLineUnlocked(QStringLiteral("FATAL std::terminate on thread %1 (%2). Typical causes: an "
                                          "exception escaping a thread or Qt slot, or a std::thread "
                                          "destroyed/reassigned while still joinable.")
            .arg(QString::number(reinterpret_cast<quintptr>(QThread::currentThreadId()), 16), what));
    }
    writeDump(QStringLiteral("std::terminate: %1").arg(what));
    std::_Exit(3);
}

void onAbortSignal(int) {
    {
        QMutexLocker locker(&g_mutex);
        appendLineUnlocked(QStringLiteral("FATAL abort() on thread %1")
            .arg(QString::number(reinterpret_cast<quintptr>(QThread::currentThreadId()), 16)));
    }
    writeDump(QStringLiteral("abort()"));
    std::_Exit(3);
}

struct Channel {
    std::atomic<long long> lastBeatMs{0};
    std::atomic<bool> armed{false};
    std::atomic<bool> reported{false};
    const char* name = "";
    long long limitMs = 0;
};
Channel g_channels[2];
std::atomic<bool> g_watchdogRunning{false};
std::thread g_watchdog;

long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

QString writeDump(const QString& reason) {
#ifdef Q_OS_WIN
    const QString dir = QFileInfo(path()).absolutePath();
    if (dir.isEmpty()) return {};
    const QString dumpFile = QDir(dir).filePath(QStringLiteral("dump_%1.dmp")
        .arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz")));
    HANDLE hFile = CreateFileW(reinterpret_cast<LPCWSTR>(dumpFile.utf16()), GENERIC_WRITE, 0, nullptr,
                               CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile == INVALID_HANDLE_VALUE) return {};
    MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile,
                      static_cast<MINIDUMP_TYPE>(MiniDumpNormal | MiniDumpWithThreadInfo),
                      nullptr, nullptr, nullptr);
    CloseHandle(hFile);
    QMutexLocker locker(&g_mutex);
    appendLineUnlocked(QStringLiteral("dump written: %1 (%2)").arg(dumpFile, reason));
    return dumpFile;
#else
    Q_UNUSED(reason);
    return {};
#endif
}

void installCrashHandlers() {
    g_previousHandler = qInstallMessageHandler(qtMessageHandler);
    std::set_terminate(onTerminate);
    std::signal(SIGABRT, onAbortSignal);
}

void beat(int channel) {
    if (channel < 0 || channel > 1) return;
    g_channels[channel].lastBeatMs.store(nowMs());
}

void setArmed(int channel, bool armed) {
    if (channel < 0 || channel > 1) return;
    g_channels[channel].lastBeatMs.store(nowMs());
    g_channels[channel].reported.store(false);
    g_channels[channel].armed.store(armed);
}

void startWatchdog() {
    if (g_watchdogRunning.exchange(true)) return;
    g_channels[0].name = "GUI thread";
    g_channels[0].limitMs = 8000;
    g_channels[1].name = "decoder thread";
    g_channels[1].limitMs = 15000;
    g_watchdog = std::thread([] {
        while (g_watchdogRunning.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            const long long now = nowMs();
            for (Channel& ch : g_channels) {
                if (!ch.armed.load()) continue;
                const long long silent = now - ch.lastBeatMs.load();
                if (silent > ch.limitMs && !ch.reported.exchange(true)) {
                    {
                        QMutexLocker locker(&g_mutex);
                        appendLineUnlocked(QStringLiteral("HANG: %1 has not responded for %2 ms")
                            .arg(QString::fromLatin1(ch.name)).arg(silent));
                    }
                    writeDump(QStringLiteral("%1 not responding").arg(QString::fromLatin1(ch.name)));
                } else if (silent <= ch.limitMs && ch.reported.load()) {
                    ch.reported.store(false);
                    QMutexLocker locker(&g_mutex);
                    appendLineUnlocked(QStringLiteral("HANG cleared: %1 responding again").arg(QString::fromLatin1(ch.name)));
                }
            }
        }
    });
}

void stopWatchdog() {
    if (!g_watchdogRunning.exchange(false)) return;
    if (g_watchdog.joinable()) g_watchdog.join();
}
}
