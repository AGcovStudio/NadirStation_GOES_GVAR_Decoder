#include "spectrumview.h"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

SpectrumView::SpectrumView(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(150);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
}

void SpectrumView::clear() {
    avg_.clear();
    update();
}

void SpectrumView::setSpectrum(const std::vector<float>& db, double sampleRate) {
    if (db.empty()) return;
    sampleRate_ = sampleRate;
    if (avg_.size() != db.size()) avg_ = db;
    else
        for (std::size_t i = 0; i < db.size(); ++i) avg_[i] += 0.3f * (db[i] - avg_[i]);
    update();
}

void SpectrumView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(250, 251, 253));
    const QRectF plot(44, 8, width() - 54, height() - 28);
    p.setPen(QColor(195, 203, 213));
    p.drawRect(plot);
    if (avg_.size() < 8) {
        p.setPen(QColor(138, 149, 163));
        p.drawText(plot, Qt::AlignCenter, QStringLiteral("The spectrum is shown here during live reception"));
        return;
    }
    // Scale: the median bin is the noise floor; show from 8 dB below it to 6 dB above the peak.
    std::vector<float> sorted = avg_;
    std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2), sorted.end());
    const float floorDb = sorted[sorted.size() / 2];
    const float peakDb = *std::max_element(avg_.begin(), avg_.end());
    const float lo = floorDb - 8.f;
    const float hi = std::max(peakDb + 6.f, lo + 30.f);

    p.setFont(QFont(QStringLiteral("Segoe UI"), 8));
    for (int k = 0; k <= 4; ++k) {                      // horizontal grid every quarter of the range
        const double y = plot.bottom() - plot.height() * k / 4.0;
        p.setPen(QColor(228, 232, 238));
        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
        p.setPen(QColor(107, 119, 133));
        p.drawText(QRectF(0, y - 8, plot.left() - 4, 16), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(lo + (hi - lo) * k / 4.0, 'f', 0));
    }
    QPainterPath path;
    const std::size_t n = avg_.size();
    const int cols = std::max(1, static_cast<int>(plot.width()));
    for (int x = 0; x < cols; ++x) {                    // one point per pixel: the maximum of the bins under it
        const std::size_t a = x * n / cols, b = std::max(a + 1, (x + 1) * n / cols);
        float v = -200.f;
        for (std::size_t i = a; i < b && i < n; ++i) v = std::max(v, avg_[i]);
        const double y = plot.bottom() - plot.height() * std::clamp((v - lo) / (hi - lo), 0.f, 1.f);
        if (x == 0) path.moveTo(plot.left(), y); else path.lineTo(plot.left() + x, y);
    }
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setPen(QPen(QColor(26, 95, 200), 1.3));
    p.drawPath(path);
    p.setRenderHint(QPainter::Antialiasing, false);

    p.setPen(QColor(107, 119, 133));
    const double half = sampleRate_ / 2e6;              // MHz
    const auto label = [&](double frac, Qt::Alignment al) {
        const double off = (frac - 0.5) * 2.0 * half;
        const QString text = centerMhz_ > 0.0 ? QString::number(centerMhz_ + off, 'f', 2) : QString::asprintf("%+.2f", off);
        const double x = plot.left() + plot.width() * frac;
        const double left = al.testFlag(Qt::AlignLeft) ? x : al.testFlag(Qt::AlignRight) ? x - 80 : x - 40;
        p.drawText(QRectF(left, plot.bottom() + 3, 80, 14), al, text + QStringLiteral(" MHz"));
    };
    label(0.0, Qt::AlignLeft | Qt::AlignVCenter);
    label(0.5, Qt::AlignHCenter | Qt::AlignVCenter);
    label(1.0, Qt::AlignRight | Qt::AlignVCenter);
    // The GVAR signal is about 3.2 MHz wide (2.111 Msym/s, roll-off 0.5): mark it around the centre.
    const double w = half > 0.0 ? std::min(1.0, 3.17 / (2.0 * half)) : 0.0;
    p.setPen(QPen(QColor(27, 127, 59, 160), 1, Qt::DashLine));
    const double x0 = plot.left() + plot.width() * (0.5 - w / 2), x1 = plot.left() + plot.width() * (0.5 + w / 2);
    p.drawLine(QPointF(x0, plot.top()), QPointF(x0, plot.bottom()));
    p.drawLine(QPointF(x1, plot.top()), QPointF(x1, plot.bottom()));
}
