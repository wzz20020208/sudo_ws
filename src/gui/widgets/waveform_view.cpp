// gui/widgets/waveform_view.cpp
// WaveformView 实现：三子图滚动波形（电压/角度/电流）+ 离屏 PNG 保存。
//
// 绘制要点：
//  - 每次 paintEvent 从控件 rect() 现算子图矩形，窗口 resize 自适应。
//  - 左右留边给 Y 刻度与「最新值」，底部留边给 X 时间标签。
//  - 自动缩放先取 min/max 再 +10% 边距；空窗口/平线以最小跨度保底，防除零。
//  - 更新走 update() 合并重画，绝不用 repaint()（会阻塞 300ms 轮询）。

#include "gui/widgets/waveform_view.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <QPixmap>
#include <QPolygonF>
#include <QRect>
#include <QString>

#include <algorithm>
#include <cmath>

namespace {

// 三个通道的颜色（标题、曲线、右上角最新值统一用）
const QColor kVoltageColor(0xd9, 0x5f, 0x02);  // 橙
const QColor kAngleColor(0x1f, 0x77, 0xb4);    // 蓝
const QColor kCurrentColor(0x2c, 0xa0, 0x2c);  // 绿

// 电压固定量程（按 24V 母线；若改用 48V 母线只改这两个常量）
constexpr double kVoltageMin = 0.0;
constexpr double kVoltageMax = 30.0;

// 自动缩放的保底最小跨度：平线/空窗口时用它，防止 Y 量程为 0 除零
constexpr double kMinAngleSpan = 2.0;    // °
constexpr double kMinCurrentSpan = 1.0;  // A

// 子图四周留边
constexpr int kLeftMargin = 44;    // Y 刻度
constexpr int kRightMargin = 52;   // 右上角最新值
constexpr int kBottomMargin = 18;  // X 时间标签
constexpr int kSubplotGap = 8;     // 子图间缝

// 网格分档数（含两端）：4 段水平 × 5 段垂直
constexpr int kGridRows = 4;
constexpr int kGridCols = 5;

}  // namespace

WaveformView::WaveformView(int max_points, double sample_interval_s, QWidget* parent)
    : QWidget(parent),
      max_points_(std::max(1, max_points)),
      sample_interval_s_(sample_interval_s > 0.0 ? sample_interval_s : 0.3) {
    // 防窗口缩到子图高度为负
    setMinimumSize(300, 300);
}

void WaveformView::append_sample(double voltage_v, double angle_deg, double iq_a) {
    if (paused_) {
        return;  // 暂停期间不接收新样本
    }
    samples_.push_back(WaveSample{voltage_v, angle_deg, iq_a});
    while (static_cast<int>(samples_.size()) > max_points_) {
        samples_.pop_front();  // 滚动窗口：超限丢最旧
    }
    update();  // 合并重画请求；绝不用 repaint()（会阻塞调用线程）
}

void WaveformView::clear_samples() {
    samples_.clear();
    update();
}

void WaveformView::set_paused(bool paused) {
    paused_ = paused;  // 只影响后续采样，当前画面保持不变，无需重画
}

bool WaveformView::is_paused() const noexcept {
    return paused_;
}

bool WaveformView::save_snapshot(const QString& file_path) {
    // grab() 离屏渲染整个控件，不依赖窗口可见性（按钮槽内调用，不在 paintEvent 里）
    const QPixmap pm = grab();
    if (pm.isNull()) {
        return false;
    }
    return pm.save(file_path, "PNG");
}

double WaveformView::channel_value(const WaveSample& s, WaveChannel ch) {
    switch (ch) {
        case WaveChannel::Voltage:
            return s.voltage_v;
        case WaveChannel::Angle:
            return s.angle_deg;
        case WaveChannel::Current:
            return s.iq_a;
    }
    return 0.0;  // 枚举穷尽后不可达，仅消除编译告警
}

void WaveformView::compute_yrange(WaveChannel ch, double& lo, double& hi) const {
    // 电压：固定量程（母线电压范围已知，自动缩放反而抖）
    if (ch == WaveChannel::Voltage) {
        lo = kVoltageMin;
        hi = kVoltageMax;
        return;
    }

    // 角度/电流：扫一遍窗口取 min/max
    double min_v = 0.0;
    double max_v = 0.0;
    bool first = true;
    for (const WaveSample& s : samples_) {
        const double v = channel_value(s, ch);
        if (first) {
            min_v = max_v = v;
            first = false;
        } else {
            min_v = std::min(min_v, v);
            max_v = std::max(max_v, v);
        }
    }

    const double min_span = (ch == WaveChannel::Angle) ? kMinAngleSpan : kMinCurrentSpan;
    if (first || max_v - min_v < min_span) {
        // 空窗口或平线：以最小跨度、当前值为中心（平线也要画在轴上），保底防除零
        const double center = first ? 0.0 : (min_v + max_v) / 2.0;
        lo = center - min_span / 2.0;
        hi = center + min_span / 2.0;
    } else {
        // +10% 边距，避免曲线顶到边框
        const double pad = (max_v - min_v) * 0.1;
        lo = min_v - pad;
        hi = max_v + pad;
    }

    // 电流要求绕 0 对称：取绝对值大的那一侧，0 线始终可见（画 0 参考线用）
    if (ch == WaveChannel::Current) {
        const double mag = std::max(std::fabs(lo), std::fabs(hi));
        lo = -mag;
        hi = mag;
    }
}

void WaveformView::draw_subplot(QPainter& painter, const QRect& plot, const QString& title,
                                WaveChannel ch, double y_min, double y_max,
                                const QColor& color) const {
    const double y_range = y_max - y_min;
    if (y_range <= 0.0 || plot.width() <= 0 || plot.height() <= 0) {
        return;  // 无效量程或窗口过小（compute_yrange 已保底，这里兜底）
    }

    // 白底
    painter.fillRect(plot, Qt::white);

    // 水平网格 + Y 刻度（上下两端也画刻度值）
    for (int i = 0; i <= kGridRows; ++i) {
        const double frac = static_cast<double>(i) / kGridRows;
        const int py = plot.top() + qRound(frac * plot.height());
        painter.setPen(QPen(QColor(0xe0, 0xe0, 0xe0), 1, Qt::DotLine));
        painter.drawLine(plot.left(), py, plot.right(), py);
        const double val = y_max - frac * y_range;
        painter.setPen(Qt::darkGray);
        painter.drawText(QRect(0, py - 7, kLeftMargin - 6, 14),
                         Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(val, 'f', 1));
    }

    // 垂直网格
    painter.setPen(QPen(QColor(0xe0, 0xe0, 0xe0), 1, Qt::DotLine));
    for (int i = 0; i <= kGridCols; ++i) {
        const double frac = static_cast<double>(i) / kGridCols;
        const int px = plot.left() + qRound(frac * plot.width());
        painter.drawLine(px, plot.top(), px, plot.bottom());
    }

    // 电流 0 参考线加深（便于看正负）
    if (ch == WaveChannel::Current && y_min < 0.0 && y_max > 0.0) {
        const int py = plot.bottom() - qRound((0.0 - y_min) / y_range * plot.height());
        painter.setPen(QPen(QColor(0xbb, 0xbb, 0xbb), 1, Qt::SolidLine));
        painter.drawLine(plot.left(), py, plot.right(), py);
    }

    // 边框（画在网格之后，边界的点线被实线盖住）
    painter.setPen(QPen(QColor(0xbb, 0xbb, 0xbb), 1));
    painter.drawRect(plot);

    // 曲线：示波器式，最新点钉右缘，step 按满窗宽算（未满窗时从右向左生长）
    const int n = static_cast<int>(samples_.size());
    if (n > 1 && max_points_ > 1) {
        const double step = static_cast<double>(plot.width()) / (max_points_ - 1);
        QPolygonF poly;
        poly.reserve(n);
        for (int i = 0; i < n; ++i) {
            const double v = channel_value(samples_[i], ch);
            const double x = plot.right() - (n - 1 - i) * step;
            const double y = plot.bottom() - (v - y_min) / y_range * plot.height();
            poly << QPointF(x, y);
        }
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(color, 1.6));
        painter.setBrush(Qt::NoBrush);
        painter.drawPolyline(poly);
    }

    // 标题（左上）与最新值（右上，通道色）
    painter.setPen(Qt::darkGray);
    painter.drawText(QRect(plot.left() + 4, plot.top() + 2, plot.width() - 8, 16),
                     Qt::AlignLeft | Qt::AlignTop, title);
    if (n > 0) {
        const char* unit = (ch == WaveChannel::Voltage) ? "V"
                           : (ch == WaveChannel::Angle) ? "°" : "A";
        painter.setPen(color);
        painter.drawText(QRect(plot.right() - 4, plot.top() + 2, kRightMargin - 6, 16),
                         Qt::AlignRight | Qt::AlignTop,
                         QString("%1 %2").arg(channel_value(samples_.back(), ch), 0, 'f', 2)
                             .arg(unit));
    }
}

void WaveformView::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), QColor(0xf5, 0xf5, 0xf5));  // 面板底色

    // 三个子图上下三等分，底部留 X 时间标签区
    const int plot_width = width() - kLeftMargin - kRightMargin;
    const int usable_height = height() - kBottomMargin;
    const int sub_height = (usable_height - 2 * kSubplotGap) / 3;
    if (plot_width <= 0 || sub_height <= 0) {
        return;  // 窗口太小，无从绘制
    }

    double v_lo = 0.0, v_hi = 0.0;
    double a_lo = 0.0, a_hi = 0.0;
    double c_lo = 0.0, c_hi = 0.0;
    compute_yrange(WaveChannel::Voltage, v_lo, v_hi);
    compute_yrange(WaveChannel::Angle, a_lo, a_hi);
    compute_yrange(WaveChannel::Current, c_lo, c_hi);

    const QRect plots[3] = {
        QRect(kLeftMargin, 0, plot_width, sub_height),
        QRect(kLeftMargin, sub_height + kSubplotGap, plot_width, sub_height),
        QRect(kLeftMargin, 2 * (sub_height + kSubplotGap), plot_width, sub_height),
    };
    draw_subplot(painter, plots[0], "电压 (V)", WaveChannel::Voltage, v_lo, v_hi,
                 kVoltageColor);
    draw_subplot(painter, plots[1], "角度 (°)", WaveChannel::Angle, a_lo, a_hi,
                 kAngleColor);
    draw_subplot(painter, plots[2], "转矩电流 (A)", WaveChannel::Current, c_lo, c_hi,
                 kCurrentColor);

    // 只在下子图下方画 X 时间标签：-60s / -30s / 0s（满窗时最左 = -total_s）
    const double total_s = sample_interval_s_ * (max_points_ - 1);
    const int bottom_plot = plots[2].bottom();
    painter.setPen(Qt::darkGray);
    for (int i = 0; i <= 2; ++i) {
        const double frac = static_cast<double>(i) / 2.0;  // 0 / 0.5 / 1.0
        const int px = plots[2].left() + qRound(frac * plot_width);
        painter.drawText(QRect(px - 20, bottom_plot + 2, 40, kBottomMargin - 2),
                         Qt::AlignHCenter | Qt::AlignTop,
                         QString("%1 s").arg(-total_s * frac, 0, 'f', 0));
    }
}
