// gvar_cli - run a recording through the GVAR receive chain without the window.
//   gvar_cli <recording> [sample rate in Hz] [output folder]
// Prints the link statistics and writes the images the same way the application does.
#include "gvarengine.h"
#include "iqreader.h"
#include "qtpath.h"

#include <QCoreApplication>
#include <QDir>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    if (argc < 2) {
        std::fprintf(stderr, "usage: gvar_cli <recording .cs8|.cu8|.cs16|.cf32|.wav> [sample rate Hz] [output folder]\n");
        return 2;
    }
    const double manualRate = argc > 2 ? std::atof(argv[2]) : 0.0;
    const QString outDir = argc > 3 ? QString::fromLocal8Bit(argv[3]) : QDir::current().filePath(QStringLiteral("gvar_out"));

    gvarapp::IqReader reader;
    std::string error;
    if (!reader.open(toFsPath(QString::fromLocal8Bit(argv[1])), manualRate, error, manualRate > 0.0)) {
        std::fprintf(stderr, "cannot open: %s\n", error.c_str());
        return 1;
    }
    const auto info = reader.info();
    std::printf("%s, %.3f MS/s (%s), %.1f s\n", gvarapp::formatName(info.format), info.sampleRate / 1e6, info.rateSource.c_str(),
                static_cast<double>(info.frames) / info.sampleRate);

    nadirstation::gvar::Engine engine;
    QDir().mkpath(outDir);
    engine.setOutputRoot(outDir);
    if (!engine.configure(info.sampleRate, 2111360.0, 0.5f, 0.02f)) {
        std::fprintf(stderr, "sample rate too low: GVAR needs at least 2.75 MS/s\n");
        return 1;
    }
    engine.reset();
    std::vector<std::complex<float>> block;
    int observations = 0;
    auto count = [&] {
        for (auto& done : engine.takeCompleted()) {
            if (!done.paths.isEmpty()) {
                ++observations;
                std::printf("observation %s: %lld files, first %s\n", done.stamp.toLocal8Bit().constData(),
                            static_cast<long long>(done.paths.size()), done.paths.first().toLocal8Bit().constData());
            }
        }
    };
    while (reader.read(block, 65536) > 0) {
        while (engine.queuedBlocks() > 6) std::this_thread::sleep_for(std::chrono::milliseconds(2));   // let the assembler keep up
        engine.process(block);
        count();
    }
    engine.finish();
    count();
    std::printf("syncs %llu, block headers ok %llu / bad %llu, lines %llu, SNR %.1f dB, observations %d\n",
                static_cast<unsigned long long>(engine.syncs()), static_cast<unsigned long long>(engine.blocksGood()),
                static_cast<unsigned long long>(engine.blocksBad()), static_cast<unsigned long long>(engine.linesStored()),
                engine.snrDb(), observations);
    return engine.syncs() > 0 ? 0 : 3;
}
