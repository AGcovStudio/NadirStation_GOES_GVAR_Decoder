#include "mainwindow.h"

#include "appinfo.h"
#include "constellationwidget.h"
#include "iqreader.h"
#include "qtpath.h"
#include "rtlsdrreceiver.h"
#include "spectrumview.h"

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QDoubleSpinBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMimeData>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QResizeEvent>
#include <QSettings>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QStatusBar>
#include <QUrl>
#include <QVBoxLayout>

namespace {

bool isSupportedRecording(const QString& path) {
    gvarapp::IqFormat f;
    return gvarapp::formatForPath(toFsPath(path), f);
}

QString humanSize(qint64 bytes) {
    const double mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
    return mb >= 1024.0 ? QStringLiteral("%1 GB").arg(mb / 1024.0, 0, 'f', 2) : QStringLiteral("%1 MB").arg(mb, 0, 'f', 1);
}

QLabel* valueLabel() {
    auto* l = new QLabel(QStringLiteral("—"));
    l->setObjectName(QStringLiteral("Value"));
    l->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return l;
}

QLabel* hint(const QString& text) {
    auto* l = new QLabel(text);
    l->setObjectName(QStringLiteral("Muted"));
    l->setWordWrap(true);
    return l;
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(QStringLiteral("%1 %2").arg(QString::fromUtf8(appinfo::kProductName), QString::fromUtf8(appinfo::kVersion)));
    setWindowIcon(QIcon(QStringLiteral(":/icon.png")));
    setAcceptDrops(true);
    resize(1240, 860);
    setMinimumSize(1200, 820);
    buildUi();

    worker_ = new GvarWorker;
    worker_->moveToThread(&thread_);
    connect(this, &MainWindow::startFileRequested, worker_, &GvarWorker::runFile);
    connect(worker_, &GvarWorker::inputInfo, this, [this](const QString& text, double) { inputInfo_->setText(text); });
    connect(worker_, &GvarWorker::logLine, this, &MainWindow::onLog);
    connect(worker_, &GvarWorker::progress, this, &MainWindow::onProgress);
    connect(worker_, &GvarWorker::statsChanged, this, &MainWindow::onStats);
    connect(worker_, &GvarWorker::constellation, this,
            [this](const std::vector<std::complex<float>>& points) { constellation_->setPoints(points); });
    connect(worker_, &GvarWorker::spectrum, this,
            [this](const std::vector<float>& db, double rate) { spectrum_->setSpectrum(db, rate); });
    connect(worker_, &GvarWorker::livePreview, this, &MainWindow::onLivePreview);
    connect(worker_, &GvarWorker::observationDone, this, &MainWindow::onObservationDone);
    connect(worker_, &GvarWorker::telemetry, this, &MainWindow::onTelemetry);
    connect(worker_, &GvarWorker::finished, this, &MainWindow::onFinished);
    thread_.start();

    QSettings settings;
    outputEdit_->setText(settings.value(QStringLiteral("outputDir"), outputRoot()).toString());
    rateSpin_->setValue(settings.value(QStringLiteral("manualRateMsps"), 3.2).toDouble());
    freqSpin_->setValue(settings.value(QStringLiteral("liveFreqMHz"), 1685.7).toDouble());
    const int rateIndex = liveRate_->findData(settings.value(QStringLiteral("liveRateHz"), 2880000).toInt());
    if (rateIndex >= 0) liveRate_->setCurrentIndex(rateIndex);
    gainSpin_->setValue(settings.value(QStringLiteral("liveGainDb"), 40.0).toDouble());
    deviceSpin_->setValue(settings.value(QStringLiteral("liveDevice"), 0).toInt());
    biasTee_->setChecked(settings.value(QStringLiteral("liveBiasTee"), false).toBool());
    if (settings.value(QStringLiteral("mode"), QStringLiteral("file")).toString() == QLatin1String("live")) liveRadio_->setChecked(true);
    onModeChanged();
    statusBar()->showMessage(QStringLiteral("Ready: choose a baseband recording, or switch to live RTL-SDR reception."));
}

MainWindow::~MainWindow() {
    if (rtl_) rtl_->stop();
    if (worker_) worker_->requestStop();
    thread_.quit();
    thread_.wait();
    delete worker_;   // its thread has finished, so deleting it from here is safe
}

bool MainWindow::liveMode() const { return liveRadio_ && liveRadio_->isChecked(); }

void MainWindow::buildUi() {
    auto* central = new QWidget;
    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(12, 12, 12, 8);
    root->setSpacing(12);

    // ---- left: where the signal comes from ----------------------------------------------------------------
    auto* left = new QWidget;
    left->setFixedWidth(344);
    auto* leftLayout = new QVBoxLayout(left);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(10);

    auto* title = new QLabel(QStringLiteral("GOES GVAR Decoder"));
    title->setObjectName(QStringLiteral("AppTitle"));
    auto* subtitle = hint(QStringLiteral("Baseband recording or live RTL-SDR → GVAR imager data → five-channel cloud images"));
    leftLayout->addWidget(title);
    leftLayout->addWidget(subtitle);

    auto* inputBox = new QGroupBox(QStringLiteral("1  Signal source"));
    auto* inputLayout = new QVBoxLayout(inputBox);
    auto* modeRow = new QHBoxLayout;
    fileRadio_ = new QRadioButton(QStringLiteral("Baseband recording"));
    liveRadio_ = new QRadioButton(QStringLiteral("Live RTL-SDR"));
    fileRadio_->setChecked(true);
    modeRow->addWidget(fileRadio_);
    modeRow->addWidget(liveRadio_);
    inputLayout->addLayout(modeRow);

    inputStack_ = new QStackedWidget;
    // page 0: recording
    auto* filePage = new QWidget;
    auto* fileLayout = new QVBoxLayout(filePage);
    fileLayout->setContentsMargins(0, 4, 0, 0);
    auto* inputRow = new QHBoxLayout;
    inputEdit_ = new QLineEdit;
    inputEdit_->setPlaceholderText(QStringLiteral("Drop a .cs8 / .cu8 / .cs16 / .cf32 / .wav file here"));
    auto* browseIn = new QPushButton(QStringLiteral("Browse…"));
    inputRow->addWidget(inputEdit_, 1);
    inputRow->addWidget(browseIn);
    fileLayout->addLayout(inputRow);
    inputInfo_ = hint(QStringLiteral("No file selected yet."));
    fileLayout->addWidget(inputInfo_);
    auto* rateRow = new QHBoxLayout;
    manualRate_ = new QCheckBox(QStringLiteral("Set the sample rate manually"));
    rateSpin_ = new QDoubleSpinBox;
    rateSpin_->setDecimals(6);
    rateSpin_->setRange(0.5, 100.0);
    rateSpin_->setValue(3.2);
    rateSpin_->setSuffix(QStringLiteral(" MS/s"));
    rateSpin_->setEnabled(false);
    rateRow->addWidget(manualRate_);
    rateRow->addWidget(rateSpin_, 1);
    fileLayout->addLayout(rateRow);
    fileLayout->addWidget(hint(QStringLiteral("cs8 / cu8 / cs16 / cf32 files carry no sample rate: the rate in a .txt next to the file is used first, then \"...SPS\" in the file name, then the value set here. WAV files use the rate in their header. The sample rate must be at least 2.75 MS/s.")));
    fileLayout->addStretch(1);
    inputStack_->addWidget(filePage);

    // page 1: RTL-SDR
    auto* livePage = new QWidget;
    auto* liveGrid = new QGridLayout(livePage);
    liveGrid->setContentsMargins(0, 4, 0, 0);
    liveGrid->setHorizontalSpacing(8);
    liveGrid->setVerticalSpacing(6);
    freqSpin_ = new QDoubleSpinBox;
    freqSpin_->setDecimals(3);
    freqSpin_->setRange(24.0, 1766.0);
    freqSpin_->setSingleStep(0.1);
    freqSpin_->setSuffix(QStringLiteral(" MHz"));
    freqSpin_->setValue(1685.7);
    liveRate_ = new QComboBox;
    liveRate_->addItem(QStringLiteral("2.88 MS/s (recommended)"), 2880000);
    liveRate_->addItem(QStringLiteral("3.00 MS/s"), 3000000);
    liveRate_->addItem(QStringLiteral("3.20 MS/s (RTL-SDR limit)"), 3200000);
    gainSpin_ = new QDoubleSpinBox;
    gainSpin_->setDecimals(1);
    gainSpin_->setRange(0.0, 49.6);
    gainSpin_->setSingleStep(1.0);
    gainSpin_->setSuffix(QStringLiteral(" dB"));
    gainSpin_->setValue(40.0);
    deviceSpin_ = new QSpinBox;
    deviceSpin_->setRange(0, 15);
    biasTee_ = new QCheckBox(QStringLiteral("Enable Bias-T (powers an active antenna / LNA)"));
    const auto row = [&](int r, const QString& name, QWidget* w) {
        auto* n = new QLabel(name);
        n->setObjectName(QStringLiteral("Muted"));
        liveGrid->addWidget(n, r, 0);
        liveGrid->addWidget(w, r, 1);
    };
    row(0, QStringLiteral("Frequency"), freqSpin_);
    row(1, QStringLiteral("Sample rate"), liveRate_);
    row(2, QStringLiteral("Gain"), gainSpin_);
    row(3, QStringLiteral("Device index"), deviceSpin_);
    liveGrid->addWidget(biasTee_, 4, 0, 1, 2);
    liveGrid->addWidget(hint(QStringLiteral("Bias-T is off by default: enable it only if the antenna or LNA really needs 5 V; on a device that does not expect power it can cause damage.\n"
                                            "An RTL-SDR at about 3 MS/s is close to the limit of USB: a poor port or cable loses samples and leaves lines missing in the image; "
                                            "use a USB port directly on the motherboard, avoid hubs. rtlsdr.dll must be in the program folder.")), 5, 0, 1, 2);
    liveGrid->setRowStretch(6, 1);
    liveGrid->setColumnStretch(1, 1);
    inputStack_->addWidget(livePage);
    inputLayout->addWidget(inputStack_);
    leftLayout->addWidget(inputBox);

    auto* outBox = new QGroupBox(QStringLiteral("2  Output folder"));
    auto* outLayout = new QVBoxLayout(outBox);
    auto* outRow = new QHBoxLayout;
    outputEdit_ = new QLineEdit;
    auto* browseOut = new QPushButton(QStringLiteral("Browse…"));
    outRow->addWidget(outputEdit_, 1);
    outRow->addWidget(browseOut);
    outLayout->addLayout(outRow);
    openOutBtn_ = new QPushButton(QStringLiteral("Open output folder"));
    outLayout->addWidget(openOutBtn_);
    leftLayout->addWidget(outBox);

    leftLayout->addStretch(1);
    startBtn_ = new QPushButton(QStringLiteral("Start decoding"));
    startBtn_->setObjectName(QStringLiteral("Primary"));
    startBtn_->setMinimumHeight(38);
    progress_ = new QProgressBar;
    progress_->setRange(0, 1000);
    progress_->setValue(0);
    progress_->setTextVisible(false);
    progress_->setFixedHeight(8);
    leftLayout->addWidget(startBtn_);
    leftLayout->addWidget(progress_);

    // ---- right: what is happening and what came out -------------------------------------------------------
    auto* right = new QWidget;
    auto* rightLayout = new QVBoxLayout(right);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(10);

    auto* topRow = new QHBoxLayout;
    topRow->setSpacing(10);
    auto* constBox = new QGroupBox(QStringLiteral("Constellation"));
    auto* constLayout = new QVBoxLayout(constBox);
    constellation_ = new ConstellationWidget;
    constellation_->setAnnotation(QStringLiteral("BPSK"), QStringLiteral("GOES GVAR · differential coding"));
    constellation_->setFixedSize(288, 288);
    constLayout->addWidget(constellation_);
    topRow->addWidget(constBox);

    auto* previewBox = new QGroupBox(QStringLiteral("Live preview"));
    auto* previewLayout = new QVBoxLayout(previewBox);
    previewLabel_ = new QLabel(QStringLiteral("Waiting for image data…"));
    previewLabel_->setAlignment(Qt::AlignCenter);
    previewLabel_->setFixedSize(288, 288);
    previewLabel_->setObjectName(QStringLiteral("Preview"));
    previewCaption_ = new QLabel;
    previewCaption_->setObjectName(QStringLiteral("Muted"));
    previewLayout->addWidget(previewLabel_);
    previewLayout->addWidget(previewCaption_);
    topRow->addWidget(previewBox);

    auto* statBox = new QGroupBox(QStringLiteral("Link status"));
    auto* grid = new QGridLayout(statBox);
    grid->setHorizontalSpacing(14);
    grid->setVerticalSpacing(8);
    const auto addStat = [&](int r, const QString& name, QLabel*& value) {
        auto* n = new QLabel(name);
        n->setObjectName(QStringLiteral("Muted"));
        value = valueLabel();
        grid->addWidget(n, r, 0);
        grid->addWidget(value, r, 1);
    };
    addStat(0, QStringLiteral("Block sync"), lockValue_);
    addStat(1, QStringLiteral("Carrier quality"), qualityValue_);
    addStat(2, QStringLiteral("Frequency offset"), offsetValue_);
    addStat(3, QStringLiteral("SNR (estimate)"), snrValue_);
    addStat(4, QStringLiteral("Header CRC ok / bad"), blockValue_);
    addStat(5, QStringLiteral("Scan lines received"), lineValue_);
    addStat(6, QStringLiteral("Spacecraft ID (block 0)"), satValue_);
    addStat(7, QStringLiteral("Spacecraft time"), timeValue_);
    grid->setRowStretch(8, 1);
    grid->setColumnStretch(1, 1);
    topRow->addWidget(statBox, 1);
    rightLayout->addLayout(topRow);

    auto* specBox = new QGroupBox(QStringLiteral("Spectrum"));
    auto* specLayout = new QVBoxLayout(specBox);
    spectrum_ = new SpectrumView;
    spectrum_->setFixedHeight(140);
    specLayout->addWidget(spectrum_);
    rightLayout->addWidget(specBox);

    auto* productBox = new QGroupBox(QStringLiteral("Observations written (double-click to open the folder)"));
    auto* productLayout = new QVBoxLayout(productBox);
    products_ = new QListWidget;
    productLayout->addWidget(products_);
    rightLayout->addWidget(productBox, 1);

    auto* logBox = new QGroupBox(QStringLiteral("Log"));
    auto* logLayout = new QVBoxLayout(logBox);
    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(2000);
    log_->setFixedHeight(100);
    logLayout->addWidget(log_);
    rightLayout->addWidget(logBox);

    root->addWidget(left);
    root->addWidget(right, 1);
    setCentralWidget(central);

    // ---- bottom strip: credit and agreement ---------------------------------------------------------------
    auto* credit = new QLabel(QStringLiteral("Developer: %1 · v%2").arg(QString::fromUtf8(appinfo::kDeveloper), QString::fromUtf8(appinfo::kVersion)));
    credit->setObjectName(QStringLiteral("Credit"));
    auto* agreementBtn = new QPushButton(QStringLiteral("Licence and disclaimer"));
    agreementBtn->setObjectName(QStringLiteral("Link"));
    agreementBtn->setFlat(true);
    agreementBtn->setCursor(Qt::PointingHandCursor);
    auto* aboutBtn = new QPushButton(QStringLiteral("About"));
    aboutBtn->setObjectName(QStringLiteral("Link"));
    aboutBtn->setFlat(true);
    aboutBtn->setCursor(Qt::PointingHandCursor);
    statusBar()->addPermanentWidget(credit);
    statusBar()->addPermanentWidget(agreementBtn);
    statusBar()->addPermanentWidget(aboutBtn);
    statusBar()->setSizeGripEnabled(false);

    connect(browseIn, &QPushButton::clicked, this, &MainWindow::browseInput);
    connect(browseOut, &QPushButton::clicked, this, &MainWindow::browseOutput);
    connect(openOutBtn_, &QPushButton::clicked, this, &MainWindow::openOutputFolder);
    connect(startBtn_, &QPushButton::clicked, this, &MainWindow::toggleRun);
    connect(agreementBtn, &QPushButton::clicked, this, &MainWindow::showAgreement);
    connect(aboutBtn, &QPushButton::clicked, this, &MainWindow::showAbout);
    connect(inputEdit_, &QLineEdit::editingFinished, this, &MainWindow::inspectInput);
    connect(fileRadio_, &QRadioButton::toggled, this, &MainWindow::onModeChanged);
    connect(manualRate_, &QCheckBox::toggled, this, [this](bool on) {
        rateSpin_->setEnabled(on);
        inspectInput();
    });
    connect(rateSpin_, &QDoubleSpinBox::editingFinished, this, &MainWindow::inspectInput);
    connect(freqSpin_, &QDoubleSpinBox::valueChanged, this, [this](double mhz) { spectrum_->setCenterMHz(mhz); });
    connect(products_, &QListWidget::itemActivated, this, [](QListWidgetItem* item) {
        const QString dir = item->data(Qt::UserRole).toString();
        if (!dir.isEmpty()) QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
    });
}

QString MainWindow::outputRoot() const {
    const QString typed = outputEdit_ ? outputEdit_->text().trimmed() : QString();
    if (!typed.isEmpty()) return typed;
    return QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).filePath(QStringLiteral("NadirStation_GOES_GVAR"));
}

void MainWindow::onModeChanged() {
    const bool live = liveMode();
    inputStack_->setCurrentIndex(live ? 1 : 0);
    progress_->setVisible(!live);
    spectrum_->setCenterMHz(live ? freqSpin_->value() : 0.0);
    if (!running_) startBtn_->setText(live ? QStringLiteral("Start receiving") : QStringLiteral("Start decoding"));
}

void MainWindow::browseInput() {
    QSettings settings;
    const QString start = settings.value(QStringLiteral("inputDir"), QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Choose a baseband recording"), start,
                                                      QStringLiteral("Baseband recordings (*.cs8 *.cu8 *.u8 *.iq *.cs16 *.cf32 *.fc32 *.cfile *.wav);;All files (*)"));
    if (path.isEmpty()) return;
    settings.setValue(QStringLiteral("inputDir"), QFileInfo(path).absolutePath());
    setInputPath(path);
}

void MainWindow::browseOutput() {
    const QString dir = QFileDialog::getExistingDirectory(this, QStringLiteral("Choose the output folder"), outputRoot());
    if (!dir.isEmpty()) outputEdit_->setText(QDir::toNativeSeparators(dir));
}

void MainWindow::openOutputFolder() {
    const QString dir = outputRoot();
    QDir().mkpath(dir);
    QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
}

void MainWindow::setInputPath(const QString& path) {
    fileRadio_->setChecked(true);
    inputEdit_->setText(QDir::toNativeSeparators(path));
    inspectInput();
}

// Read just the header of the chosen file and say what it is, so mistakes show before the long run.
void MainWindow::inspectInput() {
    if (running_) return;
    const QString path = inputEdit_->text().trimmed();
    inputValid_ = false;
    if (path.isEmpty()) {
        inputInfo_->setText(QStringLiteral("No file selected yet."));
        return;
    }
    if (!QFileInfo(path).isFile()) {
        inputInfo_->setText(QStringLiteral("File not found."));
        return;
    }
    gvarapp::IqReader reader;
    std::string error;
    const bool preferManual = manualRate_->isChecked();
    if (!reader.open(toFsPath(path), rateSpin_->value() * 1e6, error, preferManual)) {
        inputInfo_->setText(QString::fromUtf8(error.c_str()));
        return;
    }
    const gvarapp::IqInfo info = reader.info();
    const bool isWav = (info.format == gvarapp::IqFormat::Wav);
    manualRate_->setEnabled(!isWav);
    if (isWav && manualRate_->isChecked()) {
        QSignalBlocker block(manualRate_);
        manualRate_->setChecked(false);
        rateSpin_->setEnabled(false);
    }
    const bool rateUnknown = (!isWav && info.rateSource == "manual" && !preferManual);
    if (rateUnknown && !manualRate_->isChecked()) {
        QSignalBlocker block(manualRate_);
        manualRate_->setChecked(true);
        rateSpin_->setEnabled(true);
    } else if (!rateUnknown && !preferManual) {
        QSignalBlocker block(rateSpin_);
        rateSpin_->setValue(info.sampleRate / 1e6);   // show what was detected
    }
    QString text = QStringLiteral("%1 · %2 MS/s (%3) · %4 s · %5")
                       .arg(QString::fromUtf8(gvarapp::formatName(info.format)))
                       .arg(info.sampleRate / 1e6, 0, 'f', 3)
                       .arg(QString::fromUtf8(info.rateSource.c_str()))
                       .arg(static_cast<double>(info.frames) / info.sampleRate, 0, 'f', 1)
                       .arg(humanSize(QFileInfo(path).size()));
    if (rateUnknown) text += QStringLiteral("\nThe file does not state its sample rate; the value above is used for now. Check it before starting.");
    if (info.sampleRate < GvarWorker::kMinSampleRate)
        text += QStringLiteral("\nSample rate too low: GOES GVAR needs at least 2.75 MS/s.");
    inputInfo_->setText(text);
    inputValid_ = info.sampleRate >= GvarWorker::kMinSampleRate;
}

void MainWindow::saveSettings() {
    QSettings settings;
    settings.setValue(QStringLiteral("outputDir"), outputEdit_->text());
    settings.setValue(QStringLiteral("manualRateMsps"), rateSpin_->value());
    settings.setValue(QStringLiteral("liveFreqMHz"), freqSpin_->value());
    settings.setValue(QStringLiteral("liveRateHz"), liveRate_->currentData().toInt());
    settings.setValue(QStringLiteral("liveGainDb"), gainSpin_->value());
    settings.setValue(QStringLiteral("liveDevice"), deviceSpin_->value());
    settings.setValue(QStringLiteral("liveBiasTee"), biasTee_->isChecked());
    settings.setValue(QStringLiteral("mode"), liveMode() ? QStringLiteral("live") : QStringLiteral("file"));
}

void MainWindow::toggleRun() {
    if (running_) {
        if (stopping_) return;
        stopping_ = true;
        startBtn_->setEnabled(false);
        startBtn_->setText(QStringLiteral("Stopping…"));
        onLog(QStringLiteral("Stop requested; saving what has been received…"));
        if (runningLive_) {
            if (rtl_) rtl_->stop();                                          // joins the receiver thread
            QMetaObject::invokeMethod(worker_, "endLive", Qt::QueuedConnection);   // runs after the queued sample blocks
        } else {
            worker_->requestStop();
        }
        return;
    }
    if (liveMode()) startLive(); else startFile();
}

void MainWindow::startFile() {
    const QString path = inputEdit_->text().trimmed();
    if (path.isEmpty() || !QFileInfo(path).isFile() || !isSupportedRecording(path)) {
        QMessageBox::information(this, QStringLiteral("No recording selected"),
                                 QStringLiteral("Choose a .cs8, .cu8, .cs16, .cf32 or .wav recording first."));
        return;
    }
    inspectInput();
    if (!inputValid_) {
        QMessageBox::warning(this, QStringLiteral("Recording cannot be used"), inputInfo_->text());
        return;
    }
    const QString out = outputRoot();
    outputEdit_->setText(QDir::toNativeSeparators(out));
    saveSettings();

    log_->clear();
    products_->clear();
    previewLabel_->setPixmap(QPixmap());
    previewLabel_->setText(QStringLiteral("Waiting for image data…"));
    previewCaption_->clear();
    lastPreview_ = QImage();
    progress_->setValue(0);
    onStats(GvarStats{});
    spectrum_->clear();
    runningLive_ = false;
    setRunning(true);
    emit startFileRequested(path, rateSpin_->value() * 1e6, manualRate_->isChecked(), out);
}

void MainWindow::startLive() {
    const double rate = liveRate_->currentData().toDouble();
    const double freq = freqSpin_->value() * 1e6;
    const QString out = outputRoot();
    outputEdit_->setText(QDir::toNativeSeparators(out));
    saveSettings();

    if (!rtl_) {
        rtl_ = new RtlSdrReceiver(this);
        // Direct connection: the block is handed to the decoder thread from the receiver's own thread, which is
        // what lets the decoder drop blocks (and say so) instead of building an unbounded queue.
        connect(rtl_, &RtlSdrReceiver::samplesReady, worker_,
                [this](const std::shared_ptr<std::vector<std::complex<float>>>& s) { worker_->pushSamples(s); },
                Qt::DirectConnection);
        connect(rtl_, &RtlSdrReceiver::statusMessage, this, [this](const QString& text) {
            rtlLastStatus_ = text;
            onLog(text);
        });
        connect(rtl_, &RtlSdrReceiver::deviceConnected, this, [this](bool on) { rtlConnected_ = on; });
        connect(rtl_, &RtlSdrReceiver::finished, this, &MainWindow::onRtlEnded);
    }

    log_->clear();
    products_->clear();
    previewLabel_->setPixmap(QPixmap());
    previewLabel_->setText(QStringLiteral("Waiting for image data…"));
    previewCaption_->clear();
    lastPreview_ = QImage();
    onStats(GvarStats{});
    spectrum_->clear();
    spectrum_->setCenterMHz(freqSpin_->value());
    rtlConnected_ = false;
    rtlLastStatus_.clear();
    runningLive_ = true;
    setRunning(true);

    bool ok = false;
    QMetaObject::invokeMethod(worker_, "beginLive", Qt::BlockingQueuedConnection, Q_RETURN_ARG(bool, ok),
                              Q_ARG(double, rate), Q_ARG(QString, out));
    if (!ok) return;                                    // finished(false, reason) is on its way and resets the window
    rtl_->setParameters(freq, rate, rate, gainSpin_->value(), static_cast<unsigned>(deviceSpin_->value()), biasTee_->isChecked());
    rtl_->setSpectrumSettings(2048, nadirstation::dsp::SpectrumWindow::Hann);
    rtl_->start();
}

// The receiver stopped without being asked to (no device, no driver, unplugged): wrap up what was received.
void MainWindow::onRtlEnded() {
    if (!running_ || !runningLive_ || stopping_) return;
    stopping_ = true;
    startBtn_->setEnabled(false);
    startBtn_->setText(QStringLiteral("Stopping…"));
    if (!rtlConnected_) {
        const QString why = rtlLastStatus_.isEmpty() ? QStringLiteral("The RTL-SDR is not open.") : rtlLastStatus_;
        QMessageBox::warning(this, QStringLiteral("Cannot receive"), why);
    }
    QMetaObject::invokeMethod(worker_, "endLive", Qt::QueuedConnection);
}

void MainWindow::setRunning(bool running) {
    running_ = running;
    stopping_ = false;
    startBtn_->setEnabled(true);
    const bool live = liveMode();
    startBtn_->setText(running ? QStringLiteral("Stop") : (live ? QStringLiteral("Start receiving") : QStringLiteral("Start decoding")));
    fileRadio_->setEnabled(!running);
    liveRadio_->setEnabled(!running);
    inputEdit_->setEnabled(!running);
    outputEdit_->setEnabled(!running);
    manualRate_->setEnabled(!running);
    rateSpin_->setEnabled(!running && manualRate_->isChecked());
    freqSpin_->setEnabled(!running);
    liveRate_->setEnabled(!running);
    gainSpin_->setEnabled(!running);
    deviceSpin_->setEnabled(!running);
    biasTee_->setEnabled(!running);
    statusBar()->showMessage(running ? (live ? QStringLiteral("Receiving…") : QStringLiteral("Decoding…")) : QStringLiteral("Ready"));
}

void MainWindow::onLog(const QString& text) {
    log_->appendPlainText(QStringLiteral("[%1] %2").arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")), text));
}

void MainWindow::onProgress(int permille) {
    progress_->setValue(permille);
    if (running_ && !runningLive_) statusBar()->showMessage(QStringLiteral("Decoding… %1%").arg(permille / 10.0, 0, 'f', 1));
}

void MainWindow::onStats(const GvarStats& s) {
    lockValue_->setText(s.locked ? QStringLiteral("Locked") : QStringLiteral("Searching"));
    lockValue_->setStyleSheet(s.locked ? QStringLiteral("color:#1b7f3b;") : QStringLiteral("color:#a86b00;"));
    qualityValue_->setText(QStringLiteral("%1%").arg(s.lockQuality * 100.0f, 0, 'f', 0));
    offsetValue_->setText(QStringLiteral("%1 Hz").arg(s.carrierHz, 0, 'f', 0));
    snrValue_->setText(QStringLiteral("%1 dB").arg(s.snrDb, 0, 'f', 1));
    blockValue_->setText(QStringLiteral("%1 / %2").arg(s.blocksGood).arg(s.blocksBad));
    lineValue_->setText(QString::number(s.lines));
    if (running_ && runningLive_)
        statusBar()->showMessage(s.droppedIqBlocks > 0
                                     ? QStringLiteral("Receiving… decoding cannot keep up, %1 IQ blocks dropped").arg(s.droppedIqBlocks)
                                     : QStringLiteral("Receiving…"));
}

void MainWindow::onTelemetry(const QString& spacecraft, const QString& time) {
    satValue_->setText(spacecraft);
    timeValue_->setText(time);
}

void MainWindow::onLivePreview(const QImage& image, const QString& stamp) {
    if (image.isNull()) return;
    lastPreview_ = image.width() > 1400 ? image.scaledToWidth(1400, Qt::SmoothTransformation) : image;
    previewCaption_->setText(QStringLiteral("Observation %1").arg(stamp));
    updatePreview();
}

void MainWindow::updatePreview() {
    if (lastPreview_.isNull()) return;
    const QSize box = previewLabel_->size() - QSize(8, 8);
    if (box.width() < 20 || box.height() < 20) return;
    previewLabel_->setPixmap(QPixmap::fromImage(lastPreview_.scaled(box, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    updatePreview();
}

void MainWindow::onObservationDone(const QString& stamp, const QStringList& paths) {
    if (paths.isEmpty()) return;
    const QString dir = QFileInfo(paths.first()).absolutePath();
    auto* item = new QListWidgetItem(QStringLiteral("%1  ·  %2 file(s)").arg(stamp).arg(paths.size()));
    item->setData(Qt::UserRole, dir);
    item->setToolTip(QDir::toNativeSeparators(dir) + QLatin1Char('\n') + paths.join(QLatin1Char('\n')));
    products_->addItem(item);
    onLog(QStringLiteral("Observation %1 saved: %2").arg(stamp, QDir::toNativeSeparators(dir)));
}

void MainWindow::onFinished(bool ok, const QString& summary) {
    setRunning(false);
    runningLive_ = false;
    onLog(summary);
    statusBar()->showMessage(summary.section(QLatin1Char('\n'), 0, 0));
    if (!ok) QMessageBox::warning(this, QStringLiteral("Not started"), summary);
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event) {
    if (running_ || !event->mimeData()->hasUrls()) return;
    for (const QUrl& url : event->mimeData()->urls())
        if (url.isLocalFile() && isSupportedRecording(url.toLocalFile())) {
            event->acceptProposedAction();
            return;
        }
}

void MainWindow::dropEvent(QDropEvent* event) {
    for (const QUrl& url : event->mimeData()->urls())
        if (url.isLocalFile() && isSupportedRecording(url.toLocalFile())) {
            setInputPath(url.toLocalFile());
            return;
        }
}

// The agreement is accepted in the installer; this just opens the same text (EULA.txt, installed next to the program).
void MainWindow::showAgreement() {
    const QString path = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("EULA.txt"));
    if (QFileInfo::exists(path) && QDesktopServices::openUrl(QUrl::fromLocalFile(path))) return;
    QMessageBox::information(this, QStringLiteral("Licence and disclaimer"),
                             QStringLiteral("EULA.txt not found. Reinstall the software, or read the full licence on the project page."));
}

void MainWindow::showAbout() {
    QMessageBox::about(this, QStringLiteral("About"),
        QStringLiteral("<h3>%1</h3>"
                       "<p>Version %2<br>Developer: %3<br>Licence: GPL-3.0<br>%4</p>"
                       "<p>Decodes the GVAR imager data of GOES-N/O/P satellites (GOES-13/14/15): BPSK demodulation → block sync → scan-line assembly → five-channel cloud images. "
                       "The signal can come from a baseband recording (.cs8 / .cu8 / .cs16 / .cf32 / .wav) or a live RTL-SDR. The software only receives; it never transmits.</p>"
                       "<p>This is a hobbyist learning and research project, not an operational weather system, and is not affiliated with or endorsed by NOAA or NASA. Please read the licence and disclaimer before use.</p>"
                       "<p style='color:#8a919c'>Third party: Qt 6 (LGPL v3). The RTL-SDR is accessed through a user-supplied rtlsdr.dll (librtlsdr), which this software does not include.</p>")
            .arg(QString::fromUtf8(appinfo::kProductName), QString::fromUtf8(appinfo::kVersion), QString::fromUtf8(appinfo::kDeveloper),
                 QStringLiteral("<a href=\"%1\">%1</a>").arg(QString::fromUtf8(appinfo::kProjectUrl))));
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (running_) {
        const auto answer = QMessageBox::question(this, QStringLiteral("Still running"),
                                                  QStringLiteral("Reception / decoding is still running. Quitting first saves what has been received, which can take up to a minute. Quit anyway?"));
        if (answer != QMessageBox::Yes) {
            event->ignore();
            return;
        }
        if (runningLive_) {
            if (rtl_) rtl_->stop();
            QMetaObject::invokeMethod(worker_, "endLive", Qt::BlockingQueuedConnection);   // saves what was received
        } else {
            worker_->requestStop();
        }
    }
    saveSettings();
    event->accept();
}
