// spectrumview.h - a small power-spectrum plot, so the signal can be found when tuning an RTL-SDR.
#pragma once
#include <QWidget>
#include <vector>

class SpectrumView : public QWidget {
    Q_OBJECT
public:
    explicit SpectrumView(QWidget* parent = nullptr);
    void setSpectrum(const std::vector<float>& db, double sampleRate);   // smoothed over time
    void setCenterMHz(double mhz) { centerMhz_ = mhz; update(); }
    void clear();
protected:
    void paintEvent(QPaintEvent*) override;
private:
    std::vector<float> avg_;
    double sampleRate_ = 0.0;
    double centerMhz_ = 0.0;
};
