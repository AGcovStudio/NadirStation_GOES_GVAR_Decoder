// mainwindow.h - the whole window of the GOES GVAR decoder: choose a recording or an RTL-SDR, press start,
// watch it lock, open the images.
#pragma once
#include "gvarworker.h"

#include <QImage>
#include <QMainWindow>
#include <QThread>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QStackedWidget;
class ConstellationWidget;
class RtlSdrReceiver;
class SpectrumView;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

signals:
    void startFileRequested(const QString& inputPath, double manualRateHz, bool preferManual, const QString& outputDir);

protected:
    void closeEvent(QCloseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void browseInput();
    void browseOutput();
    void openOutputFolder();
    void toggleRun();
    void inspectInput();
    void onModeChanged();
    void showAgreement();
    void showAbout();
    void onLog(const QString& text);
    void onProgress(int permille);
    void onStats(const GvarStats& stats);
    void onLivePreview(const QImage& image, const QString& stamp);
    void onObservationDone(const QString& stamp, const QStringList& paths);
    void onTelemetry(const QString& spacecraft, const QString& time);
    void onFinished(bool ok, const QString& summary);
    void onRtlEnded();

private:
    void buildUi();
    void setInputPath(const QString& path);
    void setRunning(bool running);
    void updatePreview();
    void startFile();
    void startLive();
    void saveSettings();
    bool liveMode() const;
    QString outputRoot() const;

    // input
    QRadioButton* fileRadio_ = nullptr;
    QRadioButton* liveRadio_ = nullptr;
    QStackedWidget* inputStack_ = nullptr;
    QLineEdit* inputEdit_ = nullptr;
    QLabel* inputInfo_ = nullptr;
    QCheckBox* manualRate_ = nullptr;
    QDoubleSpinBox* rateSpin_ = nullptr;
    QDoubleSpinBox* freqSpin_ = nullptr;
    QComboBox* liveRate_ = nullptr;
    QDoubleSpinBox* gainSpin_ = nullptr;
    QSpinBox* deviceSpin_ = nullptr;
    QCheckBox* biasTee_ = nullptr;
    QLineEdit* outputEdit_ = nullptr;
    QPushButton* startBtn_ = nullptr;
    QPushButton* openOutBtn_ = nullptr;
    QProgressBar* progress_ = nullptr;
    // results
    ConstellationWidget* constellation_ = nullptr;
    SpectrumView* spectrum_ = nullptr;
    QLabel* lockValue_ = nullptr;
    QLabel* qualityValue_ = nullptr;
    QLabel* offsetValue_ = nullptr;
    QLabel* snrValue_ = nullptr;
    QLabel* blockValue_ = nullptr;
    QLabel* lineValue_ = nullptr;
    QLabel* satValue_ = nullptr;
    QLabel* timeValue_ = nullptr;
    QLabel* previewLabel_ = nullptr;
    QLabel* previewCaption_ = nullptr;
    QListWidget* products_ = nullptr;
    QPlainTextEdit* log_ = nullptr;

    QThread thread_;
    GvarWorker* worker_ = nullptr;
    RtlSdrReceiver* rtl_ = nullptr;
    QImage lastPreview_;
    bool running_ = false;
    bool stopping_ = false;
    bool inputValid_ = false;
    bool runningLive_ = false;
    bool rtlConnected_ = false;
    QString rtlLastStatus_;
};
