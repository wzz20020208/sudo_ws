// gui/widgets/waveform_view.hpp
// 通用实时波形控件：四路滚动曲线（电压 / 转速 / 角度 / 电流），分四个子图各用各的 Y 轴，
// 可把当前画面保存成 PNG 图片。供 motor_full_control 等轮询 GUI 复用。
//
// 设计：
//  - 纯 QPainter 自绘，不依赖 Qt5Charts（零新增依赖）；无 Q_OBJECT / moc，
//    paintEvent 是 QWidget 虚函数即可。
//  - 数据缓冲 std::deque<WaveSample>，滚动窗口：append 超过 max_points 丢最旧。
//  - 四路量纲差异大，分四个子图；电压固定量程，转速/角度/电流自动缩放（实现见 .cpp）。
//  - X 轴示波器式：最新点钉在右缘，旧点向左推移。
//  - samples_ 只允许主线程（GUI 轮询线程）访问，不加锁 —— append 在窗口线程内被
//    poll() 调用，paintEvent 也在窗口线程，二者天然串行。
#pragma once

#include <QWidget>

#include <deque>

class QPaintEvent;
class QPainter;
class QRect;
class QString;

// 一次采样的四路数据（同一时刻对齐）
struct WaveSample {
    double voltage_v = 0.0;  // 母线电压（V）
    double speed_dps = 0.0;  // 实际转速（°/s）
    double angle_deg = 0.0;  // 解包后的累计角度（°）
    double iq_a = 0.0;       // 转矩电流（A）
};

class WaveformView : public QWidget {
public:
    explicit WaveformView(int max_points = 200, double sample_interval_s = 0.3,
                          QWidget* parent = nullptr);

    void append_sample(double voltage_v, double speed_dps, double angle_deg,
                       double iq_a);  // 仅主线程调用
    void clear_samples();                                                 // 清空缓冲，不清量程基准
    void set_paused(bool paused);                                         // 暂停时不接收新样本
    bool is_paused() const noexcept;

    // 把当前画面离屏渲染成 PNG 存盘（窗口不可见也有效）；失败返回 false。
    // 非 const：内部调用 QWidget::grab()，Qt5 中该函数不是 const 成员。
    bool save_snapshot(const QString& file_path);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    enum class WaveChannel { Voltage, Speed, Angle, Current };

    // 画一个子图：白底 + 灰网格 + Y 刻度 + 曲线 + 标题 + 右上角最新值
    void draw_subplot(QPainter& painter, const QRect& plot, const QString& title,
                      WaveChannel ch, double y_min, double y_max, const QColor& color) const;
    // 按通道算 Y 量程（电压固定，角度/电流自动缩放，空窗口/平线保底防除零）
    void compute_yrange(WaveChannel ch, double& lo, double& hi) const;
    static double channel_value(const WaveSample& s, WaveChannel ch);

    int max_points_;           // 滚动窗口容量（点数）
    double sample_interval_s_; // 采样间隔（秒，用于 X 轴时间刻度）
    bool paused_ = false;      // 暂停时不接收新样本
    std::deque<WaveSample> samples_;  // 滚动窗口，超限丢最旧
};
