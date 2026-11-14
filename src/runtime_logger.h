#pragma once
#include <QString>

namespace nadirstation::log {
void init();
void write(const QString& message);

// Pushes anything buffered out to the file. For lines that exist to be read after a fault, where
// waiting for the thirty-two-line boundary may never happen.
void flush();
QString path();

// Logs and dumps what the unhandled-exception filter cannot see: std::terminate (an exception that
// escaped a thread, a std::thread destroyed or reassigned while still joinable), abort(), and Qt
// warnings and fatal errors. Call once, after init(), from the GUI thread.
void installCrashHandlers();

// Hang watchdog. Each watched thread calls beat() regularly; a background thread logs (once per
// stall) and writes a minidump when a watched thread has not beaten for longer than its limit.
// Channel 0 is the GUI thread, 1 the decoder thread. A channel is watched only while armed.
void beat(int channel);
void setArmed(int channel, bool armed);
void startWatchdog();
void stopWatchdog();

// Writes a minidump of the whole process (all thread stacks) next to the log and returns its path.
QString writeDump(const QString& reason);
}
