// gui/widgets/waveform_view.hpp
// 通用波形控件：四路曲线（电压 / 转速 / 角度 / 电流），分四个子图各用各的 Y 轴。
// 支持实时滚动（最新钉右缘）、时间轴缩放（最小 10ms）、拖动平移回看、CSV 历史加载回放。
//
// 设计：
//  - 纯 QPainter 自绘，不依赖 Qt5Charts（零新增依赖）；无 Q_OBJECT / moc，
//    paintEvent 是 QWidget 虚函数即可。
//  - 数据缓冲 std::deque<WaveSample>，每样本带 t_s（相对数据源原点秒数）。
//    实时模式由内部 steady_clock 自记，超 max_points 丢最旧；回放模式不裁剪。
//  - 拖动平移：实时模式往左拖即暂停跟随（detached_），新样本继续进缓冲但视窗
//    冻结，拖回最右缘自动恢复跟随；回放模式自由平移；两模式都钳制在数据范围内。
//  - X 轴为「可见时间窗」：view_span_s_（宽）+ view_right_s_（右缘时间），
//    实时右缘锚定最新样本（示波器式），回放 fit-all 后由缩放改变。
//  - samples_ 只允许主线程（GUI 轮询线程）访问，不加锁 —— append 在窗口线程内被
//    poll() 调用，paintEvent 也在窗口线程，二者天然串行。

#pragma once

#include <QPoint>
#include <QString>
#include <QWidget>

#include <chrono>
#include <deque>
#include <vector>

class QPaintEvent;
class QPainter;
class QRect;
class QMouseEvent;
class QWheelEvent;

// 一次采样的五路数据（同一时刻对齐；t_s 必须首位，聚合初始化按字段序）
struct WaveSample {
    double t_s = 0.0;   // 显示 X 坐标：相对数据原点秒数（实时=距 live 起点，回放=CSV 折算后）
    double voltage_v = 0.0;  // 母线电压（V）
    double speed_dps = 0.0;  // 实际转速（°/s）
    double angle_deg = 0.0;  // 解包后的累计角度（°）
    double iq_a = 0.0;       // 转矩电流（A）
};

// 一条曲线：系列 0 恒为实时/第一份历史，系列 i≥1 为叠加的其它 CSV
struct WaveSeries {
    QString label;  // 图例名（实时="实时"，历史=文件名）
    std::deque<WaveSample> samples;
};

class WaveformView : public QWidget {
public:
    explicit WaveformView(int max_points = 200, double sample_interval_s = 0.3,
                          QWidget* parent = nullptr);

    void append_sample(double voltage_v, double speed_dps, double angle_deg,
                       double iq_a);  // 仅主线程调用；回放/暂停时丢弃
    void clear_samples();             // 清空缓冲并回实时（等价 back_to_live）
    void set_paused(bool paused);     // 暂停时不接收新样本
    bool is_paused() const noexcept;

    // 加载录制 CSV 作为一个新系列叠加显示（可多次调用，按现实时间戳对齐）：
    //  - 新列 7 列（motor_id,t_s,t_epoch,voltage_v,speed_dps,angle_deg,iq_a）：t_epoch 为
    //    Unix epoch 秒，加载时折算 t_s = t_epoch - origin_epoch_，与其它带 epoch 的系列同步；
    //  - 旧列 6 列（无 t_epoch）：无法按现实时间同步，按其自身相对 t_s 显示。
    // 角度为 0x9C 原始 int16 回绕值，内部解包。首次调用清空现有系列并进入回放模式
    // （append 忽略、不裁剪、fit-all 显示）；失败返回 false 并把原因写进 *error（可为空）。
    bool add_csv(const QString& file_path, QString* error = nullptr);
    // 退出回放回实时：清空所有系列、重置时间原点、恢复满窗。
    void back_to_live();

    void zoom_in();            // 以视窗中心放大 1.25×
    void zoom_out();           // 以视窗中心缩小 1/1.25×
    bool is_replay() const noexcept { return replay_; }

    // 只读访问系列 0 缓冲（离线测试断言用；实时数据仍随时间增长；无系列时返回空队列）
    const std::deque<WaveSample>& samples() const noexcept {
        static const std::deque<WaveSample> kEmpty;
        return series_.empty() ? kEmpty : series_[0].samples;
    }
    int series_count() const noexcept { return static_cast<int>(series_.size()); }
    // 只读访问指定系列缓冲（离线测试断言多系列对齐用；index 越界返回空队列）
    const std::deque<WaveSample>& series_samples(int index) const noexcept {
        static const std::deque<WaveSample> kEmpty;
        return (index >= 0 && index < static_cast<int>(series_.size()))
                   ? series_[index].samples
                   : kEmpty;
    }

    // 把当前画面离屏渲染成 PNG 存盘（窗口不可见也有效）；失败返回 false。
    // 非 const：内部调用 QWidget::grab()，Qt5 中该函数不是 const 成员。
    bool save_snapshot(const QString& file_path);

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;  // 滚轮以光标时间为锚缩放
    void mousePressEvent(QMouseEvent* event) override;    // 按下开始拖动平移
    void mouseMoveEvent(QMouseEvent* event) override;     // 拖动中平移视窗
    void mouseReleaseEvent(QMouseEvent* event) override;  // 松开结束拖动

private:
    enum class WaveChannel { Voltage, Speed, Angle, Current };

    // 画一个子图：白底 + 灰网格 + Y 刻度 + 曲线 + 标题 + 右上角最新值
    void draw_subplot(QPainter& painter, const QRect& plot, const QString& title,
                      WaveChannel ch, double y_min, double y_max, const QColor& color) const;
    // 按通道算 Y 量程（四路都只扫可见窗口自动缩放，取整到 1/2/5 刻度步长，空/平线保底防除零）
    void compute_yrange(WaveChannel ch, double& lo, double& hi) const;
    static double channel_value(const WaveSample& s, WaveChannel ch);

    void zoom_around(double t_anchor, double factor);  // 以 t_anchor 为不动锚缩放
    void pan_view(double dt_s);  // 平移视窗（dt>0 向新数据方向），钳制到数据范围
    double default_span_s() const noexcept;            // 满窗宽 = interval*(max_points-1)
    double data_spacing_s() const noexcept;            // 实时按系列 0 算；回放数据天然 10ms
    double t_lo_s() const noexcept { return view_right_s_ - view_span_s_; }
    static double tick_step_for(double span, double plot_width);  // X 刻度步长选择
    static QColor series_color(int index);  // 系列 0 用通道色，i≥1 用叠加调色板色

    int max_points_;           // 实时滚动窗口容量（点数）
    double sample_interval_s_; // 采样间隔（秒，满窗宽的基准）
    bool paused_ = false;      // 暂停时不接收新样本
    std::vector<WaveSeries> series_;  // 系列缓冲（系列 0 实时裁剪 / 回放各系列不裁剪）
    double origin_epoch_ = 0.0;       // 现实时间同步基准：首个带 t_epoch 系列的首样本 epoch

    std::chrono::steady_clock::time_point live_origin_;  // live 时间原点（首拍才起）
    bool live_origin_set_ = false;                       // 首拍是否已建立原点
    bool replay_ = false;        // 回放模式：append 忽略、不裁剪
    double view_span_s_ = 0.0;   // 可见时间窗宽度（默认满窗 59.7s，最小 10ms）
    double view_right_s_ = 0.0;  // 可见窗右缘（数据相对时间）
    bool detached_ = false;      // 实时回看中：append 不再钉右缘，拖回最右缘恢复
    bool dragging_ = false;      // 鼠标左键按住拖动中
    QPoint drag_last_pos_;       // 上一拖动位置（算像素位移）
};
