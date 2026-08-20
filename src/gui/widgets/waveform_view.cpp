// gui/widgets/waveform_view.cpp
// WaveformView 实现：四子图滚动波形（电压/转速/角度/电流）+ 时间轴缩放 + CSV 历史回放。
//
// 绘制要点：
//  - 每次 paintEvent 从控件 rect() 现算子图矩形，窗口 resize 自适应。
//  - 左右留边给 Y 刻度与「最新值」，底部留边给 X 时间刻度。
//  - X 轴为「可见时间窗」[view_right - view_span, view_right]：实时右缘锚定最新样本
//    （示波器式），回放 fit-all 后由滚轮/按钮缩放；只画窗口内点，防子分辨率噪线。
//  - 自动缩放 Y 只统计可见窗口（回放大数据不压扁局部细节）；四路全自适应，
//    量程与刻度取整到 1/2/5 步长（好数刻度、小数位随步长自适应）；空/平线保底防除零。
//  - 更新走 update() 合并重画，绝不用 repaint()（会阻塞调用线程）。

#include "gui/widgets/waveform_view.hpp"

#include "gui/widgets/angle_unwrap.hpp"

#include <QFileInfo>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPixmap>
#include <QPolygonF>
#include <QRect>
#include <QString>
#include <QWheelEvent>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace {

// 四个通道的颜色（标题、曲线、右上角最新值统一用）
const QColor kVoltageColor(0xd9, 0x5f, 0x02);  // 橙
const QColor kSpeedColor(0x94, 0x63, 0xd2);    // 紫
const QColor kAngleColor(0x1f, 0x77, 0xb4);    // 蓝
const QColor kCurrentColor(0x2c, 0xa0, 0x2c);  // 绿

// 自动缩放的保底最小跨度：平线/空窗口时用它，防止 Y 量程为 0 除零（四路全自适应）
constexpr double kMinVoltageSpan = 4.0;  // V（母线电压波动小，跨度过小刻度全是重复数）
constexpr double kMinSpeedSpan = 10.0;   // °/s（双向绕 0 对称，跨度过小正负都贴边）
constexpr double kMinAngleSpan = 2.0;    // °
constexpr double kMinCurrentSpan = 1.0;  // A

// 时间轴缩放：最小时间窗 10ms（需求的最小单位）、每级缩放倍率、X 刻度最小像素间距
constexpr double kMinSpanS = 0.01;   // 秒
constexpr double kZoomFactor = 1.25;
constexpr double kMinTickPx = 80.0;
constexpr double kTickEps = 1e-9;    // 浮点刻度比较容差

// 子图四周留边
constexpr int kLeftMargin = 44;    // Y 刻度
constexpr int kRightMargin = 52;   // 右上角最新值
constexpr int kBottomMargin = 18;  // X 时间刻度
constexpr int kSubplotGap = 8;     // 子图间缝

// 网格分档数（含两端）：水平按 1/2/5 步长整数倍枚举（不固定段数），垂直固定 5 段
constexpr int kGridCols = 5;

// Y 刻度步长：从 1/2/5 × 10^k 数列取第一个 ≥ span/4 的（约 4 段、最多 6 段，刻度全是
// 好数）；span 无效时兜底 1。compute_yrange 与 draw_subplot 共用，保证两端步长一致
double nice_step_for(double span) {
    if (!(span > 0.0) || !std::isfinite(span)) {
        return 1.0;
    }
    const double target = span / 4.0;
    const double mag = std::pow(10.0, std::floor(std::log10(target)));
    for (const double m : {1.0, 2.0, 5.0}) {
        if (m * mag >= target) {
            return m * mag;
        }
    }
    return 10.0 * mag;  // target 落在 5×mag 与 10×mag 之间（如 7.5×mag）时到这
}

// 去行首尾空白（含 \r，兼容 Windows 换行的 CRLF）
std::string trim(const std::string& s) {
    const auto is_ws = [](unsigned char c) { return std::isspace(c) != 0; };
    size_t b = 0, e = s.size();
    while (b < e && is_ws(static_cast<unsigned char>(s[b]))) {
        ++b;
    }
    while (e > b && is_ws(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }
    return s.substr(b, e - b);
}

// 按逗号切分（录制端自己写的 CSV，字段内不含逗号，无需引号转义）
std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    size_t start = 0;
    for (size_t i = 0; i <= line.size(); ++i) {
        if (i == line.size() || line[i] == ',') {
            out.push_back(line.substr(start, i - start));
            start = i + 1;
        }
    }
    return out;
}

// 严格解析 double：strtod 必须消费整串且结果有限
bool parse_double(const std::string& s, double& out) {
    const char* begin = s.c_str();
    char* end = nullptr;
    out = std::strtod(begin, &end);
    if (end == begin || end != begin + s.size()) {
        return false;
    }
    return std::isfinite(out);
}

}  // namespace

WaveformView::WaveformView(int max_points, double sample_interval_s, QWidget* parent)
    : QWidget(parent),
      max_points_(std::max(1, max_points)),
      sample_interval_s_(sample_interval_s > 0.0 ? sample_interval_s : 0.3) {
    view_span_s_ = default_span_s();
    // 防窗口缩到子图高度为负
    setMinimumSize(300, 300);
}

void WaveformView::append_sample(double voltage_v, double speed_dps, double angle_deg,
                                 double iq_a) {
    if (replay_ || paused_) {
        return;  // 回放/暂停期间不接收新样本
    }
    if (!live_origin_set_) {
        live_origin_ = std::chrono::steady_clock::now();
        live_origin_set_ = true;
    }
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                   live_origin_)
                         .count();
    // 实时数据始终是系列 0（label="实时"）；首次喂数时重建该系列
    if (series_.empty()) {
        series_.push_back(WaveSeries{QStringLiteral("实时"), {}});
    }
    // t_s 必须首位（聚合初始化按字段序）
    auto& samples = series_[0].samples;
    samples.push_back(WaveSample{t, voltage_v, speed_dps, angle_deg, iq_a});
    while (static_cast<int>(samples.size()) > max_points_) {
        samples.pop_front();  // 实时滚动窗口：超限丢最旧
    }
    if (!detached_) {
        view_right_s_ = t;  // 跟随中才钉右缘；回看中视窗冻结，数据照常进缓冲
    }
    update();           // 合并重画请求；绝不用 repaint()（会阻塞调用线程）
}

void WaveformView::clear_samples() {
    back_to_live();
}

void WaveformView::back_to_live() {
    series_.clear();          // 清掉回放叠加的所有系列
    origin_epoch_ = 0.0;      // 现实时间同步基准一并复位
    replay_ = false;
    live_origin_set_ = false;   // 下个 append 重新起原点（t 从 ~0 开始）
    detached_ = false;          // 回看标记一并复位
    view_span_s_ = default_span_s();
    view_right_s_ = 0.0;
    update();
}

bool WaveformView::add_csv(const QString& file_path, QString* error) {
    std::ifstream in(file_path.toStdString());
    if (!in.is_open()) {
        if (error) {
            *error = "无法打开文件";
        }
        return false;
    }
    std::string line;
    std::getline(in, line);  // 跳过表头（新 7 列含 t_epoch / 旧 6 列不含）

    // 先整体解析到临时行，确认 epoch 列存在后才折算 t_s（同步基准需先定）
    struct Row {
        double t = 0.0;          // 文件里的相对秒数（旧 6 列直接作显示 t_s）
        bool has_epoch = false;  // 新 7 列才带现实时间戳
        double epoch = 0.0;      // Unix epoch 秒
        double v = 0.0, spd = 0.0, ang = 0.0, iq = 0.0;
    };
    std::vector<Row> rows;
    double prev_angle = 0.0;
    bool have_prev = false;
    size_t line_no = 1;
    while (std::getline(in, line)) {
        ++line_no;
        line = trim(line);
        if (line.empty()) {
            continue;  // 容忍文件尾空行
        }
        const std::vector<std::string> f = split_csv(line);
        Row r;
        if (f.size() == 7) {
            // 新列：motor_id,t_s,t_epoch,voltage_v,speed_dps,angle_deg,iq_a
            if (!parse_double(f[1], r.t) || !parse_double(f[2], r.epoch) ||
                !parse_double(f[3], r.v) || !parse_double(f[4], r.spd) ||
                !parse_double(f[5], r.ang) || !parse_double(f[6], r.iq)) {
                if (error) {
                    *error = QString("第 %1 行数值列格式错误").arg(line_no);
                }
                return false;
            }
            r.has_epoch = true;
        } else if (f.size() == 6) {
            // 旧列：motor_id,t_s,voltage_v,speed_dps,angle_deg,iq_a（无现实时间戳，无法同步）
            if (!parse_double(f[1], r.t) || !parse_double(f[2], r.v) ||
                !parse_double(f[3], r.spd) || !parse_double(f[4], r.ang) ||
                !parse_double(f[5], r.iq)) {
                if (error) {
                    *error = QString("第 %1 行数值列格式错误").arg(line_no);
                }
                return false;
            }
        } else {
            if (error) {
                *error = QString("第 %1 行应含 6/7 列，实为 %2").arg(line_no).arg(f.size());
            }
            return false;
        }
        // 角度为 0x9C 原始 int16（±32767° 回绕）：本文件首点立基准，之后逐点解包成累计角
        if (have_prev) {
            r.ang = gui::unwrap_angle(prev_angle, r.ang);
        }
        have_prev = true;
        prev_angle = r.ang;
        rows.push_back(r);
    }
    if (rows.empty()) {
        if (error) {
            *error = "文件为空或仅含表头";
        }
        return false;
    }

    // 首次加载（此前为实时）：清空实时系列进入回放；带 epoch 则以该系列首样本定同步基准
    const bool first_add = !replay_;
    if (first_add) {
        series_.clear();
        replay_ = true;  // 回放模式：append 忽略、各系列不裁剪
        if (rows.front().has_epoch) {
            origin_epoch_ = rows.front().epoch;
        }
    }

    // 折算显示 t_s：带 epoch 的按现实时间同步（t_s = epoch - origin_epoch_，不同加载顺序
    // 只整体平移、系列间相对时差不变）；旧 6 列无 epoch，按其自身相对 t_s 显示（无法同步）
    WaveSeries ws;
    ws.label = QFileInfo(file_path).fileName();  // 图例用文件名区分叠加的各路 CSV
    for (const Row& r : rows) {
        const double t = r.has_epoch ? (r.epoch - origin_epoch_) : r.t;
        ws.samples.push_back(WaveSample{t, r.v, r.spd, r.ang, r.iq});
    }
    series_.push_back(std::move(ws));

    // fit-all：窗口覆盖全系列数据（最左左缘 → 最右右缘），叠加多路后整体缩放可见
    double lo = series_[0].samples.front().t_s;
    double hi = series_[0].samples.front().t_s;
    for (const WaveSeries& s : series_) {
        lo = std::min(lo, s.samples.front().t_s);
        hi = std::max(hi, s.samples.back().t_s);
    }
    view_span_s_ = std::max(kMinSpanS, hi - lo);
    view_right_s_ = hi;
    update();
    return true;
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

double WaveformView::default_span_s() const noexcept {
    return sample_interval_s_ * static_cast<double>(max_points_ - 1);
}

double WaveformView::data_spacing_s() const noexcept {
    if (replay_) {
        return kMinSpanS;  // 回放数据由录制线程以 10ms 采，分辨率天然 10ms，最小窗即 10ms
    }
    if (series_.empty() || series_[0].samples.size() < 2) {
        return kMinSpanS;
    }
    const auto& s = series_[0].samples;
    return (s.back().t_s - s.front().t_s) / static_cast<double>(s.size() - 1);
}

QColor WaveformView::series_color(int index) {
    // 叠加系列调色板（系列 0 恒用通道色，不在此表）：红/青/粉/棕/灰，超出循环复用
    static const QColor kPalette[] = {
        QColor(0xd6, 0x27, 0x28),  // 红
        QColor(0x17, 0xbe, 0xcf),  // 青
        QColor(0xe3, 0x77, 0xc2),  // 粉
        QColor(0x8c, 0x56, 0x4b),  // 棕
        QColor(0x7f, 0x7f, 0x7f),  // 灰
    };
    return kPalette[(index - 1) % (sizeof(kPalette) / sizeof(kPalette[0]))];
}

void WaveformView::zoom_in() {
    // factor<1 使可见窗变窄（放大细节）；factor>1 为缩小视野
    zoom_around(view_right_s_ - view_span_s_ / 2.0, 1.0 / kZoomFactor);
}

void WaveformView::zoom_out() {
    zoom_around(view_right_s_ - view_span_s_ / 2.0, kZoomFactor);
}

void WaveformView::wheelEvent(QWheelEvent* event) {
    const int plot_w = width() - kLeftMargin - kRightMargin;
    if (plot_w <= 0 || view_span_s_ <= 0.0) {
        event->accept();
        return;
    }
    // Qt 5.15+ 用 position()（pos() 已弃用）；光标落在子图区外则夹到最左/最右
    const double x = std::clamp(event->position().x() - kLeftMargin, 0.0,
                                static_cast<double>(plot_w));
    const double t_anchor = view_right_s_ - view_span_s_ + x / plot_w * view_span_s_;
    const int delta = event->angleDelta().y();  // ±120 一格；上滚为正
    if (delta != 0) {
        // 上滚放大（factor<1，窗变窄）；下滚缩小
        zoom_around(t_anchor, std::pow(kZoomFactor, -delta / 120.0));
    }
    event->accept();
}

void WaveformView::mousePressEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton) {
        return;
    }
    dragging_ = true;
    drag_last_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
}

void WaveformView::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) {
        return;
    }
    const int plot_w = width() - kLeftMargin - kRightMargin;
    if (plot_w <= 0 || view_span_s_ <= 0.0 || series_.empty()) {
        return;
    }
    // 拖右（dx>0）看更早：视窗随手指反向移动，1px = view_span/plot_w 秒
    const double dt =
        -(event->pos().x() - drag_last_pos_.x()) * view_span_s_ / plot_w;
    drag_last_pos_ = event->pos();
    pan_view(dt);
}

void WaveformView::mouseReleaseEvent(QMouseEvent* event) {
    Q_UNUSED(event);
    dragging_ = false;
    unsetCursor();
}

void WaveformView::pan_view(double dt_s) {
    // 全体系列的时间范围（空缓冲无从平移）
    double t_min = 0.0, t_max = 0.0;
    bool any = false;
    for (const WaveSeries& ws : series_) {
        if (ws.samples.empty()) {
            continue;
        }
        const double lo = ws.samples.front().t_s;
        const double hi = ws.samples.back().t_s;
        if (!any) {
            t_min = lo;
            t_max = hi;
            any = true;
        } else {
            t_min = std::min(t_min, lo);
            t_max = std::max(t_max, hi);
        }
    }
    if (!any) {
        return;
    }
    // 实时模式首次向后拖（dt<0）→ 暂停跟随进入回看
    if (!replay_ && !detached_ && dt_s < 0.0) {
        detached_ = true;
    }
    const double new_right = std::clamp(view_right_s_ + dt_s, t_min, t_max);
    if (new_right == view_right_s_) {
        return;
    }
    view_right_s_ = new_right;
    // 拖回数据末端 → 恢复跟随（下一拍 append 重新钉右缘）
    if (!replay_ && detached_ && view_right_s_ >= t_max) {
        detached_ = false;
    }
    update();
}

void WaveformView::zoom_around(double t_anchor, double factor) {
    if (view_span_s_ <= 0.0) {
        return;
    }
    // 最小窗 = max(10ms, 数据分辨率)：10ms 录制可缩到 10ms，实时 300ms 数据最小 0.3s，
    // 窗口内始终 ≥1 点，避免放大到子分辨率出空窗/噪线
    const double min_span = std::max(kMinSpanS, data_spacing_s());
    // 最大窗覆盖全系列数据跨度（叠加多路后仍能拉到一次看全）
    double data_span = 0.0;
    for (const WaveSeries& ws : series_) {
        if (ws.samples.size() >= 2) {
            data_span =
                std::max(data_span, ws.samples.back().t_s - ws.samples.front().t_s);
        }
    }
    const double max_span = std::max(default_span_s(), data_span);
    const double new_span = std::clamp(view_span_s_ * factor, min_span, max_span);
    const double ratio = new_span / view_span_s_;
    view_right_s_ = t_anchor + (view_right_s_ - t_anchor) * ratio;  // 锚点时间不动
    view_span_s_ = new_span;
    update();
}

double WaveformView::tick_step_for(double span, double plot_width) {
    // 从细到粗选第一个满足「刻度像素间距 ≥ kMinTickPx」的（span≥10ms 时 0.01 即可）
    static constexpr double kSteps[] = {0.01, 0.02, 0.05, 0.1,  0.2,  0.5,  1,
                                        2,    5,    10,   30,   60,   120,  300,
                                        600};
    for (const double s : kSteps) {
        if (plot_width > 0.0 && (s / span) * plot_width >= kMinTickPx) {
            return s;
        }
    }
    return kSteps[sizeof(kSteps) / sizeof(kSteps[0]) - 1];
}

double WaveformView::channel_value(const WaveSample& s, WaveChannel ch) {
    switch (ch) {
        case WaveChannel::Voltage:
            return s.voltage_v;
        case WaveChannel::Speed:
            return s.speed_dps;
        case WaveChannel::Angle:
            return s.angle_deg;
        case WaveChannel::Current:
            return s.iq_a;
    }
    return 0.0;  // 枚举穷尽后不可达，仅消除编译告警
}

void WaveformView::compute_yrange(WaveChannel ch, double& lo, double& hi) const {
    // 四路都只统计可见窗口内的点（回放大数据时不把局部细节压扁）；
    // 多系列叠加时 Y 量程跨所有系列取，避免叠加曲线被挤出画面
    const double t_lo = view_right_s_ - view_span_s_;
    double min_v = 0.0;
    double max_v = 0.0;
    bool first = true;
    for (const WaveSeries& ws : series_) {
        for (const WaveSample& s : ws.samples) {
            if (s.t_s < t_lo || s.t_s > view_right_s_) {
                continue;
            }
            const double v = channel_value(s, ch);
            if (first) {
                min_v = max_v = v;
                first = false;
            } else {
                min_v = std::min(min_v, v);
                max_v = std::max(max_v, v);
            }
        }
    }

    double min_span = kMinCurrentSpan;
    if (ch == WaveChannel::Voltage) {
        min_span = kMinVoltageSpan;
    } else if (ch == WaveChannel::Speed) {
        min_span = kMinSpeedSpan;
    } else if (ch == WaveChannel::Angle) {
        min_span = kMinAngleSpan;
    }
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

    // 转速/电流要求绕 0 对称：取绝对值大的那一侧，0 线始终可见（画 0 参考线用）
    if (ch == WaveChannel::Current || ch == WaveChannel::Speed) {
        const double mag = std::max(std::fabs(lo), std::fabs(hi));
        lo = -mag;
        hi = mag;
    }

    // 量程向外取整到 1/2/5 步长的整数倍：Y 刻度全是好数、上下边框即首末刻度线
    //（绘图端按同一步长枚举）。取整会撑大跨度，撑大后可能换更大步长，迭代到稳定
    double step = nice_step_for(hi - lo);
    for (int i = 0; i < 8; ++i) {
        lo = std::floor(lo / step) * step;
        hi = std::ceil(hi / step) * step;
        const double next = nice_step_for(hi - lo);
        if (next == step) {
            break;
        }
        step = next;
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

    // 水平网格 + Y 刻度：compute_yrange 已把量程取整到步长整数倍，按整数倍枚举刻度
    //（上下边框必是首末刻度线）；小数位随步长自适应——大步长不带小数、小步长补足位数
    const double step = nice_step_for(y_range);
    const int decimals =
        std::max(0, static_cast<int>(std::ceil(-std::log10(step) - 1e-9)));
    const int n_lo = static_cast<int>(std::llround(y_min / step));
    const int n_hi = static_cast<int>(std::llround(y_max / step));
    for (int n = n_lo; n <= n_hi; ++n) {
        const double val = n * step;
        const int py = plot.top() + qRound((y_max - val) / y_range * plot.height());
        painter.setPen(QPen(QColor(0xe0, 0xe0, 0xe0), 1, Qt::DotLine));
        painter.drawLine(plot.left(), py, plot.right(), py);
        painter.setPen(Qt::darkGray);
        painter.drawText(QRect(0, py - 7, kLeftMargin - 6, 14),
                         Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(val, 'f', decimals));
    }

    // 垂直网格
    painter.setPen(QPen(QColor(0xe0, 0xe0, 0xe0), 1, Qt::DotLine));
    for (int i = 0; i <= kGridCols; ++i) {
        const double frac = static_cast<double>(i) / kGridCols;
        const int px = plot.left() + qRound(frac * plot.width());
        painter.drawLine(px, plot.top(), px, plot.bottom());
    }

    // 转速/电流 0 参考线加深（便于看正负）
    if ((ch == WaveChannel::Current || ch == WaveChannel::Speed) && y_min < 0.0 &&
        y_max > 0.0) {
        const int py = plot.bottom() - qRound((0.0 - y_min) / y_range * plot.height());
        painter.setPen(QPen(QColor(0xbb, 0xbb, 0xbb), 1, Qt::SolidLine));
        painter.drawLine(plot.left(), py, plot.right(), py);
    }

    // 边框（画在网格之后，边界的点线被实线盖住）
    painter.setPen(QPen(QColor(0xbb, 0xbb, 0xbb), 1));
    painter.drawRect(plot);

    // 曲线：每系列一条线，系列 0 用通道色、叠加系列用调色板色；只画可见时间窗内的点，
    // 按 t→x 映射（实时右缘=最新样本，回放随缩放）
    const double span = view_span_s_;
    const int n0 = series_.empty() ? 0 : static_cast<int>(series_[0].samples.size());
    if (span > 0.0 && !series_.empty()) {
        const double t_lo = view_right_s_ - span;
        const double px_per_s = plot.width() / span;  // span≥10ms>0，无除零
        painter.setRenderHint(QPainter::Antialiasing, true);
        for (int si = 0; si < static_cast<int>(series_.size()); ++si) {
            const std::deque<WaveSample>& samples = series_[si].samples;
            const QColor col = (si == 0) ? color : series_color(si);
            QPolygonF poly;
            poly.reserve(static_cast<int>(samples.size()));
            for (const WaveSample& s : samples) {
                if (s.t_s < t_lo || s.t_s > view_right_s_) {
                    continue;
                }
                const double v = channel_value(s, ch);
                const double x = plot.left() + (s.t_s - t_lo) * px_per_s;
                const double y = plot.bottom() - (v - y_min) / y_range * plot.height();
                poly << QPointF(x, y);
            }
            if (poly.size() >= 2) {
                painter.setPen(QPen(col, 1.6));
                painter.setBrush(Qt::NoBrush);
                painter.drawPolyline(poly);
            } else if (poly.size() == 1) {
                // 窗口内仅 1 个采样点：画实心圆点，放大到接近数据分辨率时也有痕迹
                painter.setPen(Qt::NoPen);
                painter.setBrush(col);
                painter.drawEllipse(poly.first(), 2.0, 2.0);
            }
        }
    }

    // 标题（左上）与最新值（右上，通道色，叠加时仍只取系列 0，多路靠图例区分）
    painter.setPen(Qt::darkGray);
    painter.drawText(QRect(plot.left() + 4, plot.top() + 2, plot.width() - 8, 16),
                     Qt::AlignLeft | Qt::AlignTop, title);
    if (n0 > 0) {
        const char* unit = (ch == WaveChannel::Voltage) ? "V"
                           : (ch == WaveChannel::Speed) ? "°/s"
                           : (ch == WaveChannel::Angle) ? "°" : "A";
        painter.setPen(color);
        painter.drawText(QRect(plot.right() - 4, plot.top() + 2, kRightMargin - 6, 16),
                         Qt::AlignRight | Qt::AlignTop,
                         QString("%1 %2")
                             .arg(channel_value(series_[0].samples.back(), ch), 0, 'f', 2)
                             .arg(unit));
    }
}

void WaveformView::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), QColor(0xf5, 0xf5, 0xf5));  // 面板底色

    // 多系列叠加时顶部预留图例条；四个子图在其下四等分，底部留 X 时间刻度区
    const int legend_h = (series_count() > 1 || detached_) ? 16 : 0;
    const int plot_width = width() - kLeftMargin - kRightMargin;
    const int usable_height = height() - kBottomMargin - legend_h;
    const int sub_height = (usable_height - 3 * kSubplotGap) / 4;
    if (plot_width <= 0 || sub_height <= 0) {
        return;  // 窗口太小，无从绘制
    }

    double v_lo = 0.0, v_hi = 0.0;
    double s_lo = 0.0, s_hi = 0.0;
    double a_lo = 0.0, a_hi = 0.0;
    double c_lo = 0.0, c_hi = 0.0;
    compute_yrange(WaveChannel::Voltage, v_lo, v_hi);
    compute_yrange(WaveChannel::Speed, s_lo, s_hi);
    compute_yrange(WaveChannel::Angle, a_lo, a_hi);
    compute_yrange(WaveChannel::Current, c_lo, c_hi);

    const QRect plots[4] = {
        QRect(kLeftMargin, legend_h, plot_width, sub_height),
        QRect(kLeftMargin, legend_h + sub_height + kSubplotGap, plot_width, sub_height),
        QRect(kLeftMargin, legend_h + 2 * (sub_height + kSubplotGap), plot_width,
              sub_height),
        QRect(kLeftMargin, legend_h + 3 * (sub_height + kSubplotGap), plot_width,
              sub_height),
    };
    draw_subplot(painter, plots[0], "电压 (V)", WaveChannel::Voltage, v_lo, v_hi,
                 kVoltageColor);
    draw_subplot(painter, plots[1], "转速 (°/s)", WaveChannel::Speed, s_lo, s_hi,
                 kSpeedColor);
    draw_subplot(painter, plots[2], "角度 (°)", WaveChannel::Angle, a_lo, a_hi,
                 kAngleColor);
    draw_subplot(painter, plots[3], "转矩电流 (A)", WaveChannel::Current, c_lo, c_hi,
                 kCurrentColor);

    // 图例条：● 文件名。系列 0 在四个子图里用各自通道色，此处以角度蓝作代表；
    // 叠加系列固定用调色板色，与各子图曲线一致，靠文件名区分哪路
    if (legend_h > 0) {
        int x = 4;
        for (int i = 0; i < series_count(); ++i) {
            const QColor col = (i == 0) ? kAngleColor : series_color(i);
            const QString name = series_[i].label;
            const int tw = painter.fontMetrics().horizontalAdvance(name);
            painter.setPen(col);
            painter.setBrush(col);
            painter.drawEllipse(QRect(x, legend_h / 2 - 2, 4, 4));
            painter.setPen(Qt::darkGray);
            painter.drawText(QRect(x + 7, 0, tw, legend_h),
                             Qt::AlignLeft | Qt::AlignVCenter, name);
            x += 11 + tw + 14;  // ● + 名字 + 项间距
            if (x >= width()) {
                break;  // 放不下截断，数据不受影响
            }
        }
    }

    // 回看提示条（detached_ 时 legend_h 已预留）：红字标在顶部右侧，拖回最右缘消失
    if (detached_) {
        painter.setPen(Qt::darkRed);
        painter.drawText(QRect(0, 0, width() - 6, legend_h),
                         Qt::AlignRight | Qt::AlignVCenter,
                         "回看中：实时冻结，拖到最右缘恢复跟随");
    }

    // 只在下子图下方画 X 时间刻度：单位按可见窗宽自适应（<1s→ms，<60s→s，≥60s→min），
    // 步长按 tick_step_for 取整（min 模式强制 ≥60s 才好看）；
    // 实时显示「相对当前」（-60s…0s），回放显示「相对数据起点」（0s…60s）
    const double span = view_span_s_;
    if (span > 0.0) {
        const double t_lo = view_right_s_ - span;
        double step = tick_step_for(span, plot_width);
        double scale = 1.0;  // 标签数值 = (t - anchor) * scale
        int decimals = 0;
        QString unit;
        if (span < 1.0) {
            scale = 1000.0;
            unit = "ms";
            decimals = 0;
        } else if (span < 60.0) {
            scale = 1.0;
            unit = "s";
            decimals = step < 0.1 ? 2 : step < 1.0 ? 1 : 0;
        } else {
            scale = 1.0 / 60.0;
            unit = "min";
            decimals = 1;
            step = std::max(step, 60.0);  // min 刻度稀疏化，标签才不挤
        }
        const double anchor = replay_ ? 0.0 : view_right_s_;
        const int bottom_plot = plots[3].bottom();
        painter.setPen(Qt::darkGray);
        for (double t = std::ceil(t_lo / step) * step; t <= view_right_s_ + kTickEps;
             t += step) {
            const int px = plots[3].left() + qRound((t - t_lo) / span * plot_width);
            painter.drawText(QRect(px - 40, bottom_plot + 2, 80, kBottomMargin - 2),
                             Qt::AlignHCenter | Qt::AlignTop,
                             QString("%1 %2")
                                 .arg((t - anchor) * scale, 0, 'f', decimals)
                                 .arg(unit));
        }
    }
}
