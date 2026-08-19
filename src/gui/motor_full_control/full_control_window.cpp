// gui/motor_full_control/full_control_window.cpp
// 全量电机控制窗口实现：单窗口 + 七个选项卡，覆盖协议层全部已实现指令。
//  - 轮询：QTimer 主线程 0x9A + 0x9C（走 Motor 过滤读，只读，可与控制程序并行）。
//  - 单机操作：Motor 覆盖的走 Motor（过滤），未覆盖的（0xA8/0xA9/0x70/0x94、
//    0x60~0x63、0x42/0x43、0x76）走 raw 协议 + CanComm。
//  - 所有副作用操作执行期间暂停轮询 timer，并弹确认框（默认 No）。

#include "full_control_window.hpp"

#include "gui/widgets/waveform_view.hpp"
#include "motor_can/can_comm/can_comm.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStringList>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <utility>
#include <vector>

namespace {

// Tab7 波形：窗口点数与采样间隔。300ms 一拍 → 200 点 ≈ 满窗 60s；
// 与构造器 timer_->start(300) 对齐，避免 X 轴时间刻度失真。
constexpr int kWaveformMaxPoints = 200;
constexpr double kWaveformIntervalS = 0.3;

// 一个带单位的浮点输入框
QDoubleSpinBox* make_double(QWidget* parent, double lo, double hi, int decimals,
                            const QString& suffix) {
    auto* box = new QDoubleSpinBox(parent);
    box->setRange(lo, hi);
    box->setDecimals(decimals);
    box->setSuffix(suffix);
    return box;
}

// 一个带单位的整数输入框
QSpinBox* make_spin(QWidget* parent, int lo, int hi, int value, const QString& suffix) {
    auto* box = new QSpinBox(parent);
    box->setRange(lo, hi);
    box->setValue(value);
    box->setSuffix(suffix);
    return box;
}

// 组一个 PID 参数输入框：可配置环可编辑，电流环置灰只读
QDoubleSpinBox* make_pid_spin(QWidget* parent, bool editable) {
    auto* box = new QDoubleSpinBox(parent);
    box->setRange(-1.0e6, 1.0e6);
    box->setDecimals(4);
    if (!editable) {
        box->setEnabled(false);
    }
    return box;
}

// 带副作用操作的确认框：默认选「否」，危险操作不强推
bool confirm(QWidget* parent, const QString& title, const QString& text) {
    return QMessageBox::question(parent, title, text, QMessageBox::Yes | QMessageBox::No,
                                 QMessageBox::No) == QMessageBox::Yes;
}

// 一组控件横向排布：items 拉伸均分，action 固定宽度（可为 nullptr）
QWidget* row(std::initializer_list<QWidget*> items, QWidget* action = nullptr) {
    auto* w = new QWidget;
    auto* h = new QHBoxLayout(w);
    h->setContentsMargins(0, 0, 0, 0);
    for (QWidget* item : items) {
        h->addWidget(item, 1);
    }
    if (action != nullptr) {
        h->addWidget(action);
    }
    return w;
}

// 读取按钮 + 结果标签横向排布（按钮固定，标签拉伸换行）
QWidget* row_result(QPushButton* btn, QLabel* label) {
    auto* w = new QWidget;
    auto* h = new QHBoxLayout(w);
    h->setContentsMargins(0, 0, 0, 0);
    h->addWidget(btn);
    h->addWidget(label, 1);
    return w;
}

// 0x70 运行模式枚举转显示文本（decode_mode 已保证 mode 合法）
const char* mode_text(motor_can::RunMode mode) {
    switch (mode) {
        case motor_can::RunMode::Current:
            return "电流环";
        case motor_can::RunMode::Speed:
            return "速度环";
        case motor_can::RunMode::Position:
            return "位置环";
    }
    return "未知";
}

}  // namespace

FullControlWindow::FullControlWindow(motor_can::CanComm& comm) : comm_(comm) {
    setWindowTitle("全量电机控制");
    setMinimumWidth(460);

    // 只读句柄：不归0（home_on_init=false），回复等待 100ms 保证界面灵敏
    mcfg_.home_on_init = false;
    mcfg_.reply_timeout = std::chrono::milliseconds(100);
    rebuild_motor();

    auto* layout = new QVBoxLayout(this);

    // ---- 顶部条：电机选择 + 在线状态 ----
    id_spin_ = new QSpinBox(this);
    id_spin_->setRange(1, 32);
    id_spin_->setValue(motor_id_);
    status_label_ = new QLabel("--", this);
    scan_btn_ = new QPushButton("扫描在线电机", this);
    online_label_ = new QLabel("--", this);

    auto* top = new QFormLayout;
    top->addRow("电机 ID (1~32)", id_spin_);
    top->addRow("状态", status_label_);
    top->addRow("在线", row({scan_btn_, online_label_}));
    layout->addLayout(top);

    auto* tabs = new QTabWidget(this);
    tabs->addTab(build_tab_control(), "控制");
    tabs->addTab(build_tab_waveform(), "波形");
    tabs->addTab(build_tab_status(), "状态");
    tabs->addTab(build_tab_params(), "参数");
    tabs->addTab(build_tab_encoder(), "编码器/零点");
    tabs->addTab(build_tab_system(), "系统");
    tabs->addTab(build_tab_multi(), "多机");
    layout->addWidget(tabs);

    connect(id_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
            [this](int v) {
                motor_id_ = v;
                rebuild_motor();  // 只换只读句柄，不驱动电机
            });
    connect(scan_btn_, &QPushButton::clicked, this, [this] { scan_online(); });

    // 每 300ms 轮询一次状态 + 运行数据
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this] { poll(); });
    timer_->start(300);
}

FullControlWindow::~FullControlWindow() = default;

// ---------------------------------------------------------------------------
// 通用辅助
// ---------------------------------------------------------------------------

void FullControlWindow::rebuild_motor() {
    motor_ = std::make_unique<motor_can::Motor>(comm_, static_cast<uint8_t>(motor_id_), mcfg_);
}

bool FullControlWindow::query(uint8_t cmd, motor_can::CanFrame& reply) {
    uint8_t data[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};
    motor_can::CanFrame f;
    f.id = 0x140u + motor_id_;
    f.is_extended = false;
    f.dlc = 8;
    std::copy_n(data, 8, f.data);

    return comm_.send(f) &&
           comm_.receive_by_id(0x240u + motor_id_, reply, std::chrono::milliseconds(100));
}

bool FullControlWindow::send_and_receive(const motor_can::CanFrame& frame,
                                         motor_can::CanFrame& reply) {
    return comm_.send(frame) &&
           comm_.receive_by_id(0x240u + motor_id_, reply, std::chrono::milliseconds(100));
}

void FullControlWindow::poll() {
    // 0x9A 状态：过滤读（杂帧丢弃），Motor 内部 100ms 内等命令字节匹配
    motor_can::MotorStatus st;
    if (!motor_->read_status(st)) {
        status_label_->setText(QString("电机 ID %1 无响应").arg(motor_id_));
        return;
    }
    status_label_->setText(QString("电机 ID %1 在线").arg(motor_id_));

    // 0x9A 字段连续刷新：即使后续 0x9C 无响应，电压/MOS温度/抱闸/错误这四栏仍保持常新
    rt_volt_label_->setText(QString("%1 V").arg(st.voltage_v, 0, 'f', 1));
    rt_mos_label_->setText(QString("%1 ℃").arg(st.mos_temp_c));
    rt_brake_label_->setText(st.brake_released ? "已释放" : "锁死");
    rt_error_label_->setText(st.error_state ? QString("0x%1").arg(st.error_state, 4, 16, QLatin1Char('0'))
                                            : QString("0x0000（无错误）"));

    // 0x9C 运行数据
    motor_can::MotorRunStatus rs;
    if (!motor_->read_run_status(rs)) {
        status_label_->setText(QString("电机 ID %1 在线（0x9C 无响应）").arg(motor_id_));
        return;
    }
    rt_angle_label_->setText(QString("%1 °").arg(rs.angle_deg, 0, 'f', 1));
    rt_speed_label_->setText(QString("%1 dps").arg(rs.speed_dps, 0, 'f', 0));
    rt_current_label_->setText(QString("%1 A").arg(rs.iq_a, 0, 'f', 2));
    rt_temp_label_->setText(QString("%1 ℃").arg(rs.temp_c));

    // 波形喂数：电压取 0x9A 的 st.voltage_v，转速取 0x9C 的 rs.speed_dps，角度先解
    // int16 回绕再累计（波形是单调累计角，不是 ±32767° 锯齿）；首拍只建立解包基准不累计。
    // 暂停由控件自行丢数。
    if (!angle_initialized_) {
        angle_unwrapped_ = rs.angle_deg;
        angle_initialized_ = true;
    } else {
        angle_unwrapped_ = unwrap_angle(angle_unwrapped_, rs.angle_deg);
    }
    waveform_->append_sample(st.voltage_v, rs.speed_dps, angle_unwrapped_, rs.iq_a);
}

double FullControlWindow::unwrap_angle(double prev, double cur) {
    double delta = cur - prev;
    // int16 角度 ±32767° 回绕：相邻两拍跳变超过半圈（±32768°）即判定整圈回绕，
    // 补偿 ±65536° 使累计角度连续。300ms 一拍、即使最高速也不会真有 18000° 跳变。
    if (delta > 32768.0) {
        delta -= 65536.0;
    } else if (delta < -32768.0) {
        delta += 65536.0;
    }
    return prev + delta;
}

// ---------------------------------------------------------------------------
// Tab1 控制
// ---------------------------------------------------------------------------

QWidget* FullControlWindow::build_tab_control() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    // 三环控制输入
    cur_target_box_ = make_double(page, -1.0e3, 1.0e3, 2, " A");
    spd_target_box_ = make_double(page, -1.0e6, 1.0e6, 1, " °/s");
    spd_torque_box_ = make_spin(page, 0, 255, 30, " %");
    pos_target_box_ = make_double(page, -1.0e6, 1.0e6, 2, " °");
    pos_speed_box_ = make_spin(page, 1, 3000, 30, " °/s");

    auto* cur_btn = new QPushButton("发送", page);
    auto* spd_btn = new QPushButton("发送", page);
    auto* pos_btn = new QPushButton("发送", page);

    auto* three_box = new QGroupBox("三环控制 (0xA1/0xA2/0xA4)", page);
    auto* tf = new QFormLayout(three_box);
    tf->addRow("转矩电流 / 发送", row({cur_target_box_}, cur_btn));
    tf->addRow("速度 / 限扭 / 发送", row({spd_target_box_, spd_torque_box_}, spd_btn));
    tf->addRow("位置 / 限速 / 发送", row({pos_target_box_, pos_speed_box_}, pos_btn));
    layout->addWidget(three_box);

    connect(cur_btn, &QPushButton::clicked, this, [this] { set_current(); });
    connect(spd_btn, &QPushButton::clicked, this, [this] { set_speed(); });
    connect(pos_btn, &QPushButton::clicked, this, [this] { set_position(); });

    // 单圈 / 增量 / 力控
    single_dir_box_ = new QComboBox(page);
    single_dir_box_->addItem("顺时针", 0);
    single_dir_box_->addItem("逆时针", 1);
    single_speed_box_ = make_spin(page, 1, 3000, 30, " °/s");
    single_angle_box_ = make_double(page, 0.0, 359.99, 2, " °");
    inc_delta_box_ = make_double(page, -1.0e6, 1.0e6, 2, " °");
    inc_speed_box_ = make_spin(page, 1, 3000, 30, " °/s");
    force_target_box_ = make_double(page, -1.0e6, 1.0e6, 2, " °");
    force_speed_box_ = make_spin(page, 1, 3000, 30, " °/s");
    force_torque_box_ = make_spin(page, 0, 255, 30, " %");

    auto* sang_btn = new QPushButton("发送", page);
    auto* inc_btn = new QPushButton("发送", page);
    auto* force_btn = new QPushButton("发送", page);

    auto* ext_box = new QGroupBox("单圈/增量/力控 (0xA6/0xA8/0xA9)", page);
    auto* ef = new QFormLayout(ext_box);
    ef->addRow("单圈：方向/转速/角度", row({single_dir_box_, single_speed_box_, single_angle_box_}, sang_btn));
    ef->addRow("增量：增量/转速", row({inc_delta_box_, inc_speed_box_}, inc_btn));
    ef->addRow("力控：角度/转速/限扭", row({force_target_box_, force_speed_box_, force_torque_box_}, force_btn));
    auto* ext_note = new QLabel("单圈位置（0xA6）为直驱用，X2-7 多圈模式不适用；0xA8/0xA9 回复布局同 0x9C。", page);
    ext_note->setWordWrap(true);
    ef->addRow(ext_note);
    layout->addWidget(ext_box);

    connect(sang_btn, &QPushButton::clicked, this, [this] { set_single_angle(); });
    connect(inc_btn, &QPushButton::clicked, this, [this] { set_increment(); });
    connect(force_btn, &QPushButton::clicked, this, [this] { set_force_position(); });

    // 抱闸 / 启动 / 停止 / 关闭
    auto* start_btn = new QPushButton("启动", page);
    auto* release_btn = new QPushButton("开闸", page);
    auto* stop_btn = new QPushButton("停止", page);
    auto* lock_btn = new QPushButton("锁闸", page);
    auto* off_btn = new QPushButton("关闭", page);
    auto* home_btn = new QPushButton("归0", page);

    auto* brake_box = new QGroupBox("抱闸 / 启动 / 停止 / 关闭 (0x77/0xA2/0x78/0x80/0x81)", page);
    auto* bf = new QVBoxLayout(brake_box);
    bf->addWidget(row({start_btn, release_btn, stop_btn, lock_btn, off_btn, home_btn}));
    auto* brake_note = new QLabel(
        "带抱闸电机运动前必须先「开闸」；锁闸自动先停止再锁。启动 = 开闸 + 速度闭环 0 转速"
        "（使能并锁定当前位置）；归0 = 开闸后驱动到 0°（真实运动）。",
        page);
    brake_note->setWordWrap(true);
    bf->addWidget(brake_note);
    layout->addWidget(brake_box);

    connect(start_btn, &QPushButton::clicked, this, [this] { start_motor(); });
    connect(release_btn, &QPushButton::clicked, this, [this] { brake_release(); });
    connect(stop_btn, &QPushButton::clicked, this, [this] { motor_stop(); });
    connect(lock_btn, &QPushButton::clicked, this, [this] { brake_lock(); });
    connect(off_btn, &QPushButton::clicked, this, [this] { motor_off(); });
    connect(home_btn, &QPushButton::clicked, this, [this] { motor_home(); });

    // 实时区：0x9A 状态字段（电压/MOS温度/抱闸/错误）+ 0x9C 运行数据，随轮询连续刷新
    rt_angle_label_ = new QLabel("--", page);
    rt_speed_label_ = new QLabel("--", page);
    rt_current_label_ = new QLabel("--", page);
    rt_temp_label_ = new QLabel("--", page);
    rt_volt_label_ = new QLabel("--", page);
    rt_mos_label_ = new QLabel("--", page);
    rt_brake_label_ = new QLabel("--", page);
    rt_error_label_ = new QLabel("--", page);

    auto* rt_box = new QGroupBox("实时数据 (0x9A + 0x9C)", page);
    auto* rf = new QFormLayout(rt_box);
    rf->addRow("电压 (V)", rt_volt_label_);
    rf->addRow("MOS 温度 (℃)", rt_mos_label_);
    rf->addRow("抱闸", rt_brake_label_);
    rf->addRow("错误标志", rt_error_label_);
    rf->addRow("角度 (°)", rt_angle_label_);
    rf->addRow("转速 (dps)", rt_speed_label_);
    rf->addRow("转矩电流 (A)", rt_current_label_);
    rf->addRow("温度 (℃)", rt_temp_label_);
    layout->addWidget(rt_box);

    layout->addStretch(1);
    return page;
}

void FullControlWindow::set_current() {
    timer_->stop();
    motor_can::MotorRunStatus st;
    if (!motor_->set_current(cur_target_box_->value(), &st)) {
        status_label_->setText("转矩指令失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(QString("转矩 %1 A").arg(cur_target_box_->value(), 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::set_speed() {
    timer_->stop();
    motor_can::MotorRunStatus st;
    if (!motor_->set_speed(spd_target_box_->value(),
                           static_cast<uint8_t>(spd_torque_box_->value()), &st)) {
        status_label_->setText("速度指令失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(QString("速度 %1 °/s").arg(spd_target_box_->value(), 0, 'f', 1));
    timer_->start();
}

void FullControlWindow::set_position() {
    timer_->stop();
    motor_can::MotorRunStatus st;
    if (!motor_->set_position(pos_target_box_->value(),
                              static_cast<uint16_t>(pos_speed_box_->value()), &st)) {
        status_label_->setText("位置指令失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(QString("位置 %1 °").arg(pos_target_box_->value(), 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::set_single_angle() {
    timer_->stop();
    motor_can::MotorRunStatus st;
    if (!motor_->set_single_angle_position(
            static_cast<uint8_t>(single_dir_box_->currentIndex()),
            static_cast<uint16_t>(single_speed_box_->value()), single_angle_box_->value(), &st)) {
        status_label_->setText("单圈位置失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(QString("单圈位置 %1 °").arg(single_angle_box_->value(), 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::set_increment() {
    timer_->stop();
    const motor_can::CanFrame f = motor_can::encode_increment_position(
        motor_id_, inc_delta_box_->value(), static_cast<uint16_t>(inc_speed_box_->value()));
    motor_can::CanFrame reply;
    motor_can::MotorRunStatus st;
    if (!send_and_receive(f, reply) || !motor_can::decode_run_status(reply, st)) {
        status_label_->setText("增量位置失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(QString("增量 %1 °").arg(inc_delta_box_->value(), 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::set_force_position() {
    timer_->stop();
    const motor_can::CanFrame f = motor_can::encode_force_position(
        motor_id_, force_target_box_->value(), static_cast<uint16_t>(force_speed_box_->value()),
        static_cast<uint8_t>(force_torque_box_->value()));
    motor_can::CanFrame reply;
    motor_can::MotorRunStatus st;
    if (!send_and_receive(f, reply) || !motor_can::decode_run_status(reply, st)) {
        status_label_->setText("力控位置失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(QString("力控位置 %1 °").arg(force_target_box_->value(), 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::brake_release() {
    timer_->stop();
    if (!motor_->brake_release()) {
        status_label_->setText("开闸失败：无回复或命令字节不符");
    } else {
        status_label_->setText("抱闸已释放");
    }
    timer_->start();
}

void FullControlWindow::start_motor() {
    // 启动 = 开闸 + 速度闭环 0 转速（使能并锁定当前位置，随时可发运动指令）。
    // 限扭复用「速度 / 限扭」框；0x77 后立即发 0xA2，正契合 X2-7「运动指令才真正
    // 完成释放」的怪癖，无需中间延时。两步中任一无回复则中止并提示。
    timer_->stop();
    if (!motor_->brake_release()) {
        status_label_->setText("启动失败：开闸无回复或命令字节不符");
        timer_->start();
        return;
    }
    motor_can::MotorRunStatus st;
    if (!motor_->set_speed(0.0, static_cast<uint8_t>(spd_torque_box_->value()), &st)) {
        status_label_->setText("启动失败：使能无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(QString("电机已启动：速度闭环 0 转速，限扭 %1%")
                               .arg(spd_torque_box_->value()));
    timer_->start();
}

void FullControlWindow::motor_stop() {
    timer_->stop();
    if (!motor_->stop()) {
        status_label_->setText("停止失败：无回复或命令字节不符");
    } else {
        status_label_->setText("电机已停止");
    }
    timer_->start();
}

void FullControlWindow::brake_lock() {
    timer_->stop();
    // 先停再锁，避免带转锁闸
    if (!motor_->stop() || !motor_->brake_lock()) {
        status_label_->setText("锁闸失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText("已停止并锁闸");
    timer_->start();
}

void FullControlWindow::motor_off() {
    timer_->stop();
    if (!confirm(this, "关闭输出", "关闭电机输出并清除运行状态（0x80）？")) {
        timer_->start();
        return;
    }
    if (!motor_->off()) {
        status_label_->setText("关闭失败：无回复或命令字节不符");
    } else {
        status_label_->setText("电机已关闭");
    }
    timer_->start();
}

void FullControlWindow::motor_home() {
    timer_->stop();
    if (!confirm(this, "物理归0", "将开闸并驱动电机回到 0° 位置（真实运动）？")) {
        timer_->start();
        return;
    }
    if (!motor_->home()) {
        status_label_->setText("归0失败：无回复或命令字节不符");
    } else {
        status_label_->setText("已归0");
    }
    timer_->start();
}

// ---------------------------------------------------------------------------
// Tab2 状态
// ---------------------------------------------------------------------------

QWidget* FullControlWindow::build_tab_status() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    auto* st1_btn = new QPushButton("读取", page);
    st1_result_ = new QLabel("--", page);
    st1_result_->setWordWrap(true);
    auto* st2_btn = new QPushButton("读取", page);
    st2_result_ = new QLabel("--", page);
    st2_result_->setWordWrap(true);
    auto* st3_btn = new QPushButton("读取", page);
    st3_result_ = new QLabel("--", page);
    st3_result_->setWordWrap(true);
    auto* ang_btn = new QPushButton("读取", page);
    ang_result_ = new QLabel("--", page);
    ang_result_->setWordWrap(true);
    auto* sang_btn = new QPushButton("读取", page);
    sang_result_ = new QLabel("--", page);
    sang_result_->setWordWrap(true);
    auto* enc_btn = new QPushButton("读取", page);
    enc_result_ = new QLabel("--", page);
    enc_result_->setWordWrap(true);
    auto* mode_btn = new QPushButton("读取", page);
    mode_result_ = new QLabel("--", page);
    mode_result_->setWordWrap(true);
    auto* time_btn = new QPushButton("读取", page);
    time_result_ = new QLabel("--", page);
    time_result_->setWordWrap(true);
    auto* ver_btn = new QPushButton("读取", page);
    ver_result_ = new QLabel("--", page);
    ver_result_->setWordWrap(true);
    auto* model_btn = new QPushButton("读取", page);
    model_result_ = new QLabel("--", page);
    model_result_->setWordWrap(true);

    auto* f = new QFormLayout;
    f->addRow("状态1 (0x9A)", row_result(st1_btn, st1_result_));
    f->addRow("状态2 (0x9C)", row_result(st2_btn, st2_result_));
    f->addRow("状态3 (0x9D)", row_result(st3_btn, st3_result_));
    f->addRow("多圈角度 (0x92)", row_result(ang_btn, ang_result_));
    f->addRow("单圈角度 (0x94)", row_result(sang_btn, sang_result_));
    f->addRow("单圈编码器 (0x90)", row_result(enc_btn, enc_result_));
    f->addRow("运行模式 (0x70)", row_result(mode_btn, mode_result_));
    f->addRow("运行时间 (0xB1)", row_result(time_btn, time_result_));
    f->addRow("版本日期 (0xB2)", row_result(ver_btn, ver_result_));
    f->addRow("电机型号 (0xB5)", row_result(model_btn, model_result_));
    layout->addLayout(f);

    connect(st1_btn, &QPushButton::clicked, this, [this] { read_status1(); });
    connect(st2_btn, &QPushButton::clicked, this, [this] { read_status2(); });
    connect(st3_btn, &QPushButton::clicked, this, [this] { read_status3(); });
    connect(ang_btn, &QPushButton::clicked, this, [this] { read_angle_now(); });
    connect(sang_btn, &QPushButton::clicked, this, [this] { read_single_angle(); });
    connect(enc_btn, &QPushButton::clicked, this, [this] { read_single_encoder(); });
    connect(mode_btn, &QPushButton::clicked, this, [this] { read_mode(); });
    connect(time_btn, &QPushButton::clicked, this, [this] { read_run_time(); });
    connect(ver_btn, &QPushButton::clicked, this, [this] { read_version_date(); });
    connect(model_btn, &QPushButton::clicked, this, [this] { read_motor_model(); });

    layout->addStretch(1);
    return page;
}

void FullControlWindow::read_status1() {
    timer_->stop();
    motor_can::MotorStatus st;
    if (!motor_->read_status(st)) {
        st1_result_->setText("无回复");
        timer_->start();
        return;
    }
    st1_result_->setText(QString("温度 %1 ℃ / MOS %2 ℃ / 电压 %3 V / 抱闸 %4 / 错误 0x%5")
                             .arg(st.temp_c)
                             .arg(st.mos_temp_c)
                             .arg(st.voltage_v, 0, 'f', 1)
                             .arg(st.brake_released ? "释放" : "锁死")
                             .arg(st.error_state, 4, 16, QLatin1Char('0')));
    timer_->start();
}

void FullControlWindow::read_status2() {
    timer_->stop();
    motor_can::MotorRunStatus st;
    if (!motor_->read_run_status(st)) {
        st2_result_->setText("无回复");
        timer_->start();
        return;
    }
    st2_result_->setText(QString("温度 %1 ℃ / 转矩电流 %2 A / 转速 %3 dps / 角度 %4 °")
                             .arg(st.temp_c)
                             .arg(st.iq_a, 0, 'f', 2)
                             .arg(st.speed_dps, 0, 'f', 0)
                             .arg(st.angle_deg, 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::read_status3() {
    timer_->stop();
    motor_can::MotorStatus3 st;
    if (!motor_->read_status3(st)) {
        st3_result_->setText("无回复");
        timer_->start();
        return;
    }
    st3_result_->setText(QString("温度 %1 ℃ / iA %2 A / iB %3 A / iC %4 A")
                             .arg(st.temp_c)
                             .arg(st.ia_a, 0, 'f', 2)
                             .arg(st.ib_a, 0, 'f', 2)
                             .arg(st.ic_a, 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::read_angle_now() {
    timer_->stop();
    double angle = 0.0;
    if (!motor_->read_angle(angle)) {
        ang_result_->setText("无回复");
        timer_->start();
        return;
    }
    ang_result_->setText(QString("%1 °").arg(angle, 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::read_single_angle() {
    timer_->stop();
    motor_can::CanFrame reply;
    double angle = 0.0;
    if (!query(static_cast<uint8_t>(motor_can::RhCmd::ReadSingleAngle), reply) ||
        !motor_can::decode_single_angle(reply, angle)) {
        sang_result_->setText("无回复或命令字节不符");
        timer_->start();
        return;
    }
    sang_result_->setText(QString("%1 °").arg(angle, 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::read_single_encoder() {
    timer_->stop();
    motor_can::SingleEncoder enc;
    if (!motor_->read_single_encoder(enc)) {
        enc_result_->setText("无回复");
        timer_->start();
        return;
    }
    enc_result_->setText(QString("encoder %1 / raw %2 / offset %3（脉冲）")
                             .arg(enc.encoder)
                             .arg(enc.raw)
                             .arg(enc.offset));
    timer_->start();
}

void FullControlWindow::read_mode() {
    timer_->stop();
    motor_can::CanFrame reply;
    motor_can::RunMode mode;
    if (!query(static_cast<uint8_t>(motor_can::RhCmd::ReadMode), reply) ||
        !motor_can::decode_mode(reply, mode)) {
        mode_result_->setText("无回复或命令字节不符");
        timer_->start();
        return;
    }
    mode_result_->setText(mode_text(mode));
    timer_->start();
}

void FullControlWindow::read_run_time() {
    timer_->stop();
    uint32_t time_ms = 0;
    if (!motor_->read_run_time(time_ms)) {
        time_result_->setText("无回复");
        timer_->start();
        return;
    }
    time_result_->setText(QString("%1 ms（约 %2 分钟）").arg(time_ms).arg(time_ms / 60000.0, 0, 'f', 0));
    timer_->start();
}

void FullControlWindow::read_version_date() {
    timer_->stop();
    uint32_t date = 0;
    if (!motor_->read_version_date(date)) {
        ver_result_->setText("无回复");
        timer_->start();
        return;
    }
    ver_result_->setText(QString("软件版本日期 %1").arg(date));
    timer_->start();
}

void FullControlWindow::read_motor_model() {
    timer_->stop();
    char model[8] = {0};
    if (!motor_->read_motor_model(model)) {
        model_result_->setText("无回复");
        timer_->start();
        return;
    }
    model_result_->setText(QString::fromLocal8Bit(model));
    timer_->start();
}

// ---------------------------------------------------------------------------
// Tab3 参数
// ---------------------------------------------------------------------------

QWidget* FullControlWindow::build_tab_params() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    // 三环 PID（电流环只读）
    cur_kp_box_ = make_pid_spin(page, false);
    cur_ki_box_ = make_pid_spin(page, false);
    spd_kp_box_ = make_pid_spin(page, true);
    spd_ki_box_ = make_pid_spin(page, true);
    pos_kp_box_ = make_pid_spin(page, true);
    pos_ki_box_ = make_pid_spin(page, true);
    pos_kd_box_ = make_pid_spin(page, true);

    auto* pid_box = new QGroupBox("三环 PID (0x30/0x31/0x32)", page);
    auto* pf = new QFormLayout(pid_box);
    pf->addRow("电流环 KP（只读）", cur_kp_box_);
    pf->addRow("电流环 KI（只读）", cur_ki_box_);
    pf->addRow("速度环 KP", spd_kp_box_);
    pf->addRow("速度环 KI", spd_ki_box_);
    pf->addRow("位置环 KP", pos_kp_box_);
    pf->addRow("位置环 KI", pos_ki_box_);
    pf->addRow("位置环 KD", pos_kd_box_);
    auto* read_btn = new QPushButton("读取", page);
    auto* ram_btn = new QPushButton("应用(RAM)", page);
    auto* rom_btn = new QPushButton("保存(ROM)", page);
    pf->addRow("操作", row({read_btn, ram_btn, rom_btn}));
    layout->addWidget(pid_box);

    connect(read_btn, &QPushButton::clicked, this, [this] { read_pid_all(); });
    connect(ram_btn, &QPushButton::clicked, this,
            [this] { write_pid_all(motor_can::RhCmd::PidWriteRam); });
    connect(rom_btn, &QPushButton::clicked, this,
            [this] { write_pid_all(motor_can::RhCmd::PidWriteRom); });

    // 加速度（0x42 读 / 0x43 写，RAM+ROM）
    pos_accel_box_ = make_double(page, 100.0, 60000.0, 0, " °/s²");
    pos_decel_box_ = make_double(page, 100.0, 60000.0, 0, " °/s²");
    spd_accel_box_ = make_double(page, 100.0, 60000.0, 0, " °/s²");
    spd_decel_box_ = make_double(page, 100.0, 60000.0, 0, " °/s²");

    auto* accel_box = new QGroupBox("加速度 (0x42/0x43)", page);
    auto* af = new QFormLayout(accel_box);
    af->addRow("位置规划加速度", pos_accel_box_);
    af->addRow("位置规划减速度", pos_decel_box_);
    af->addRow("速度规划加速度", spd_accel_box_);
    af->addRow("速度规划减速度", spd_decel_box_);
    auto* accel_read_btn = new QPushButton("读取", page);
    auto* accel_write_btn = new QPushButton("写入(ROM)", page);
    af->addRow("操作", row({accel_read_btn, accel_write_btn}));
    layout->addWidget(accel_box);

    connect(accel_read_btn, &QPushButton::clicked, this, [this] { read_accel_all(); });
    connect(accel_write_btn, &QPushButton::clicked, this, [this] { write_accel_all(); });

    param_result_label_ = new QLabel("--", page);
    param_result_label_->setWordWrap(true);
    layout->addWidget(param_result_label_);

    layout->addStretch(1);
    return page;
}

void FullControlWindow::read_pid_all() {
    // 7 个环参数（含电流环，仅展示）
    const std::pair<motor_can::PidIndex, QDoubleSpinBox*> fields[] = {
        {motor_can::PidIndex::CurrentKp, cur_kp_box_},
        {motor_can::PidIndex::CurrentKi, cur_ki_box_},
        {motor_can::PidIndex::SpeedKp, spd_kp_box_},
        {motor_can::PidIndex::SpeedKi, spd_ki_box_},
        {motor_can::PidIndex::PositionKp, pos_kp_box_},
        {motor_can::PidIndex::PositionKi, pos_ki_box_},
        {motor_can::PidIndex::PositionKd, pos_kd_box_},
    };
    timer_->stop();
    bool ok = true;
    for (const auto& f : fields) {
        float value = 0.0f;
        if (!motor_->read_pid(f.first, value)) {
            ok = false;
            param_result_label_->setText(QString("读取 PID(索引 0x%1) 失败")
                                             .arg(static_cast<int>(f.first), 2, 16, QLatin1Char('0')));
            break;
        }
        f.second->setValue(value);
    }
    if (ok) {
        param_result_label_->setText("PID 已读取");
    }
    timer_->start();
}

void FullControlWindow::write_pid_all(motor_can::RhCmd cmd) {
    // 只写可配置环（速度 Kp/Ki、位置 Kp/Ki/Kd），电流环只读不写
    const std::pair<motor_can::PidIndex, QDoubleSpinBox*> fields[] = {
        {motor_can::PidIndex::SpeedKp, spd_kp_box_},
        {motor_can::PidIndex::SpeedKi, spd_ki_box_},
        {motor_can::PidIndex::PositionKp, pos_kp_box_},
        {motor_can::PidIndex::PositionKi, pos_ki_box_},
        {motor_can::PidIndex::PositionKd, pos_kd_box_},
    };
    timer_->stop();
    bool ok = true;
    for (const auto& f : fields) {
        const bool done = (cmd == motor_can::RhCmd::PidWriteRam)
                              ? motor_->write_pid_ram(f.first, f.second->value())
                              : motor_->write_pid_rom(f.first, f.second->value());
        if (!done) {
            ok = false;
            param_result_label_->setText(QString("写 PID(索引 0x%1) 失败")
                                             .arg(static_cast<int>(f.first), 2, 16, QLatin1Char('0')));
            break;
        }
    }
    if (ok) {
        param_result_label_->setText(cmd == motor_can::RhCmd::PidWriteRam ? "PID 已写入 RAM（掉电丢失）"
                                                                          : "PID 已写入 ROM（掉电保存）");
    }
    timer_->start();
}

void FullControlWindow::read_accel_all() {
    const std::pair<motor_can::AccelIndex, QDoubleSpinBox*> fields[] = {
        {motor_can::AccelIndex::PositionAccel, pos_accel_box_},
        {motor_can::AccelIndex::PositionDecel, pos_decel_box_},
        {motor_can::AccelIndex::SpeedAccel, spd_accel_box_},
        {motor_can::AccelIndex::SpeedDecel, spd_decel_box_},
    };
    timer_->stop();
    bool ok = true;
    for (const auto& f : fields) {
        motor_can::CanFrame reply;
        double value = 0.0;
        if (!send_and_receive(motor_can::encode_read_accel(motor_id_, f.first), reply) ||
            !motor_can::decode_accel(reply, value)) {
            ok = false;
            param_result_label_->setText(QString("读取加速度(索引 0x%1) 失败")
                                             .arg(static_cast<int>(f.first), 2, 16, QLatin1Char('0')));
            break;
        }
        f.second->setValue(value);
    }
    if (ok) {
        param_result_label_->setText("加速度已读取");
    }
    timer_->start();
}

void FullControlWindow::write_accel_all() {
    const std::pair<motor_can::AccelIndex, QDoubleSpinBox*> fields[] = {
        {motor_can::AccelIndex::PositionAccel, pos_accel_box_},
        {motor_can::AccelIndex::PositionDecel, pos_decel_box_},
        {motor_can::AccelIndex::SpeedAccel, spd_accel_box_},
        {motor_can::AccelIndex::SpeedDecel, spd_decel_box_},
    };
    timer_->stop();
    bool ok = true;
    for (const auto& f : fields) {
        motor_can::CanFrame reply;
        double value = 0.0;
        if (!send_and_receive(motor_can::encode_write_accel(motor_id_, f.first, f.second->value()),
                              reply) ||
            !motor_can::decode_accel(reply, value)) {
            ok = false;
            param_result_label_->setText(QString("写加速度(索引 0x%1) 失败")
                                             .arg(static_cast<int>(f.first), 2, 16, QLatin1Char('0')));
            break;
        }
    }
    if (ok) {
        param_result_label_->setText("加速度已写入（RAM+ROM）");
    }
    timer_->start();
}

// ---------------------------------------------------------------------------
// Tab4 编码器 / 零点
// ---------------------------------------------------------------------------

QWidget* FullControlWindow::build_tab_encoder() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    // 读取 0x60/0x61/0x62
    enc_pos_label_ = new QLabel("--", page);
    enc_raw_label_ = new QLabel("--", page);
    enc_offset_label_ = new QLabel("--", page);
    auto* read_enc_btn = new QPushButton("读取", page);

    auto* read_box = new QGroupBox("编码器位置 (0x60/0x61/0x62)", page);
    auto* rf = new QFormLayout(read_box);
    rf->addRow("多圈位置", row_result(read_enc_btn, enc_pos_label_));
    rf->addRow("原始位置", enc_raw_label_);
    rf->addRow("零偏", enc_offset_label_);
    layout->addWidget(read_box);

    connect(read_enc_btn, &QPushButton::clicked, this, [this] { read_encoders(); });

    // 写零偏 0x63
    enc_offset_box_ = new QSpinBox(page);
    enc_offset_box_->setRange(-100000, 100000);
    enc_offset_box_->setSuffix(" 脉冲");
    auto* write_off_btn = new QPushButton("写入 (0x63)", page);

    auto* write_box = new QGroupBox("写零偏 (0x63，写 ROM)", page);
    auto* wf = new QFormLayout(write_box);
    wf->addRow("新零偏", row({enc_offset_box_}, write_off_btn));
    auto* off_note = new QLabel("零偏点作为角度 0 点；写入后需 0x76 复位才生效。", page);
    off_note->setWordWrap(true);
    wf->addRow(off_note);
    layout->addWidget(write_box);

    connect(write_off_btn, &QPushButton::clicked, this, [this] { write_encoder_offset(); });

    // 零点操作
    auto* zero_btn = new QPushButton("记当前点为零点 (0x64)", page);
    auto* clear_btn = new QPushButton("清除多圈值 (0x20/0x01)", page);

    auto* zero_box = new QGroupBox("零点操作", page);
    auto* zf = new QVBoxLayout(zero_box);
    zf->addWidget(row({zero_btn, clear_btn}));
    auto* zero_note = new QLabel(
        "记零点：把当前编码器位置记为多圈零点并写 ROM，需 0x76 复位后生效，避免电机运动时写入。\n"
        "清多圈：清零多圈计数、更新零点并保存，重启后生效。", page);
    zero_note->setWordWrap(true);
    zf->addWidget(zero_note);
    layout->addWidget(zero_box);

    connect(zero_btn, &QPushButton::clicked, this, [this] { set_zero_point(); });
    connect(clear_btn, &QPushButton::clicked, this, [this] { clear_multi_turn(); });

    // 掉电保存多圈 0x20/0x04
    power_save_check_ = new QCheckBox("多圈值掉电保存", page);
    auto* apply_ps_btn = new QPushButton("应用 (0x20/0x04)", page);

    auto* ps_box = new QGroupBox("多圈值掉电保存 (0x20/0x04)", page);
    auto* pf = new QVBoxLayout(ps_box);
    pf->addWidget(row({power_save_check_}, apply_ps_btn));
    auto* ps_note = new QLabel("开启后掉电前保存当前多圈值（重启后生效）；关闭为系统默认单圈模式。", page);
    ps_note->setWordWrap(true);
    pf->addWidget(ps_note);
    layout->addWidget(ps_box);

    connect(apply_ps_btn, &QPushButton::clicked, this, [this] { apply_power_save(); });

    enc_result_label_ = new QLabel("--", page);
    enc_result_label_->setWordWrap(true);
    layout->addWidget(enc_result_label_);

    layout->addStretch(1);
    return page;
}

void FullControlWindow::read_encoders() {
    timer_->stop();
    const std::pair<motor_can::RhCmd, QLabel*> reads[] = {
        {motor_can::RhCmd::EncoderPos, enc_pos_label_},
        {motor_can::RhCmd::EncoderRaw, enc_raw_label_},
        {motor_can::RhCmd::EncoderOffset, enc_offset_label_},
    };
    bool ok = true;
    for (const auto& r : reads) {
        motor_can::CanFrame reply;
        int32_t pos = 0;
        if (!query(static_cast<uint8_t>(r.first), reply) ||
            !motor_can::decode_encoder_position(reply, pos)) {
            r.second->setText("无回复");
            ok = false;
        } else {
            r.second->setText(QString("%1 脉冲").arg(pos));
        }
    }
    if (ok) {
        enc_result_label_->setText("编码器已读取（0x60/0x61/0x62）");
    }
    timer_->start();
}

void FullControlWindow::write_encoder_offset() {
    timer_->stop();
    const int32_t offset = static_cast<int32_t>(enc_offset_box_->value());
    if (!confirm(this, "写零偏",
                 QString("将编码器零偏写为 %1 脉冲（写 ROM，需 0x76 复位后生效）？")
                     .arg(offset))) {
        timer_->start();
        return;
    }
    motor_can::CanFrame reply;
    int32_t echoed = 0;
    if (!send_and_receive(motor_can::encode_write_encoder_offset(motor_id_, offset), reply) ||
        !motor_can::decode_encoder_position(reply, echoed)) {
        enc_result_label_->setText("写零偏失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    enc_result_label_->setText(QString("已写入零偏 %1 脉冲（0x76 复位后生效）").arg(echoed));
    timer_->start();
}

void FullControlWindow::set_zero_point() {
    timer_->stop();
    if (!confirm(this, "记零点", "将当前编码器位置记录为多圈零点并写入 ROM？\n\n"
                                 "注意：需 0x76 复位后生效；避免电机运动时写入。")) {
        timer_->start();
        return;
    }
    int32_t offset = 0;
    if (!motor_->set_zero_point(offset)) {
        enc_result_label_->setText("记零点失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    enc_result_label_->setText(QString("已记录，新零偏 = %1 脉冲（0x76 复位后生效）").arg(offset));
    timer_->start();
}

void FullControlWindow::clear_multi_turn() {
    timer_->stop();
    if (!confirm(this, "清除多圈值",
                 "清除电机多圈值：清零多圈计数、更新零点并保存（重启后生效）？")) {
        timer_->start();
        return;
    }
    if (!motor_->clear_multi_turn()) {
        enc_result_label_->setText("清多圈失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    enc_result_label_->setText("已清除多圈值（重启后生效）");
    timer_->start();
}

void FullControlWindow::apply_power_save() {
    timer_->stop();
    const bool enable = power_save_check_->isChecked();
    if (!confirm(this, "多圈掉电保存",
                 enable ? "开启多圈值掉电保存（重启后生效）？" : "关闭多圈值掉电保存？")) {
        timer_->start();
        return;
    }
    if (!motor_->set_multi_turn_power_save(enable)) {
        enc_result_label_->setText("设置失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    enc_result_label_->setText(QString("多圈值掉电保存已%1").arg(enable ? "开启" : "关闭"));
    timer_->start();
}

// ---------------------------------------------------------------------------
// Tab5 系统
// ---------------------------------------------------------------------------

QWidget* FullControlWindow::build_tab_system() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    // 0x20 功能控制开关 + 改 ID + 位置限位
    filter_check_ = new QCheckBox("CANID 滤波器使能（广播前置需失能）", page);
    auto* apply_filter_btn = new QPushButton("应用 (0x20/0x02)", page);
    error_report_check_ = new QCheckBox("错误状态发送使能", page);
    auto* apply_err_btn = new QPushButton("应用 (0x20/0x03)", page);
    new_id_box_ = new QSpinBox(page);
    new_id_box_->setRange(1, 32);
    new_id_box_->setValue(motor_id_);
    auto* change_id_btn = new QPushButton("修改 (0x20/0x05)", page);
    limit_pos_box_ = make_double(page, -1.0e4, 1.0e4, 2, " °");
    limit_pos_box_->setValue(360.0);
    limit_neg_box_ = make_double(page, -1.0e4, 1.0e4, 2, " °");
    limit_neg_box_->setValue(-360.0);
    auto* limit_btn = new QPushButton("写入限位 (0x20/0x06/0x07)", page);

    auto* fc_box = new QGroupBox("0x20 功能控制", page);
    auto* fcf = new QFormLayout(fc_box);
    fcf->addRow("滤波器", row({filter_check_}, apply_filter_btn));
    fcf->addRow("错误上报", row({error_report_check_}, apply_err_btn));
    fcf->addRow("新 CAN ID", row({new_id_box_}, change_id_btn));
    fcf->addRow("最大正/负角度", row({limit_pos_box_, limit_neg_box_}, limit_btn));
    auto* fc_note = new QLabel(
        "改 ID：按地址写入当前 ID 的电机（同 ID 多台一起改），写入后立即切到新 ID，"
        "需重启电机才持久化。限位单位按 0.01°/LSB 打包（手册未注明，待真机确认），写 ROM 立即生效。",
        page);
    fc_note->setWordWrap(true);
    fcf->addRow(fc_note);
    layout->addWidget(fc_box);

    connect(apply_filter_btn, &QPushButton::clicked, this, [this] { apply_canid_filter(); });
    connect(apply_err_btn, &QPushButton::clicked, this, [this] { apply_error_report(); });
    connect(change_id_btn, &QPushButton::clicked, this, [this] { change_can_id(); });
    connect(limit_btn, &QPushButton::clicked, this, [this] { write_position_limits(); });

    // 主动回复 0xB6
    report_cmd_box_ = new QComboBox(page);
    report_cmd_box_->addItem("多圈位置 (0x60)", 0x60);
    report_cmd_box_->addItem("原始位置 (0x61)", 0x61);
    report_cmd_box_->addItem("零偏 (0x62)", 0x62);
    report_cmd_box_->addItem("多圈角度 (0x92)", 0x92);
    report_cmd_box_->addItem("状态1 (0x9A)", 0x9A);
    report_cmd_box_->addItem("状态2 (0x9C)", 0x9C);
    report_cmd_box_->addItem("状态3 (0x9D)", 0x9D);
    report_cmd_box_->addItem("单圈编码器 (0x9E)", 0x9E);
    report_enable_check_ = new QCheckBox("使能", page);
    report_interval_box_ = make_spin(page, 1, 65535, 10, " ×10ms");
    auto* report_btn = new QPushButton("发送 (0xB6)", page);

    auto* report_box = new QGroupBox("主动回复 (0xB6)", page);
    auto* rf = new QVBoxLayout(report_box);
    rf->addWidget(row({report_cmd_box_, report_enable_check_, report_interval_box_}, report_btn));
    auto* report_note = new QLabel(
        "对 0x9A/0x9C 使能主动回复后，本窗口轮询将收不到回复（显示无响应）；"
        "使能后该指令不再回复命令，仅按间隔主动上报。无回复，成功与否需观察总线。",
        page);
    report_note->setWordWrap(true);
    rf->addWidget(report_note);
    layout->addWidget(report_box);

    connect(report_btn, &QPushButton::clicked, this, [this] { send_active_report(); });

    // 通讯中断保护 0xB3
    protect_time_box_ = new QSpinBox(page);
    protect_time_box_->setRange(0, 600000);
    protect_time_box_->setSuffix(" ms");
    auto* protect_btn = new QPushButton("写入 (0xB3)", page);

    auto* protect_box = new QGroupBox("通讯中断保护 (0xB3，写 ROM)", page);
    auto* pf = new QFormLayout(protect_box);
    pf->addRow("保护时间", row({protect_time_box_}, protect_btn));
    auto* protect_note = new QLabel(
        "通讯中断超过设定时间会自动切断输出并锁死抱闸；0=关闭。避免在电机刚启动/运动时写入。",
        page);
    protect_note->setWordWrap(true);
    pf->addRow(protect_note);
    layout->addWidget(protect_box);

    connect(protect_btn, &QPushButton::clicked, this, [this] { write_protect_time(); });

    // 系统复位 0x76
    auto* reset_btn = new QPushButton("系统复位 (0x76)", page);
    auto* reset_box = new QGroupBox("系统复位", page);
    auto* rstf = new QVBoxLayout(reset_box);
    rstf->addWidget(reset_btn);
    auto* reset_note = new QLabel("复位无回复；0x64 新零点、0x20 相关配置在复位后生效。", page);
    reset_note->setWordWrap(true);
    rstf->addWidget(reset_note);
    layout->addWidget(reset_box);

    connect(reset_btn, &QPushButton::clicked, this, [this] { reset_motor(); });

    sys_result_label_ = new QLabel("--", page);
    sys_result_label_->setWordWrap(true);
    layout->addWidget(sys_result_label_);

    layout->addStretch(1);
    return page;
}

void FullControlWindow::apply_canid_filter() {
    timer_->stop();
    const bool enable = filter_check_->isChecked();
    if (!confirm(this, "CANID 滤波器",
                 enable ? "使能 CANID 滤波器（存 FLASH）？广播 0x280 前需先失能。"
                        : "失能 CANID 滤波器（存 FLASH）？")) {
        timer_->start();
        return;
    }
    if (!motor_->set_can_id_filter(enable)) {
        sys_result_label_->setText("设置失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    sys_result_label_->setText(QString("CANID 滤波器已%1").arg(enable ? "使能" : "失能"));
    timer_->start();
}

void FullControlWindow::apply_error_report() {
    timer_->stop();
    const bool enable = error_report_check_->isChecked();
    if (!confirm(this, "错误上报",
                 enable ? "使能错误状态发送（出错后主动发 0x9A，100ms 周期）？"
                        : "失能错误状态发送？")) {
        timer_->start();
        return;
    }
    if (!motor_->set_error_report(enable)) {
        sys_result_label_->setText("设置失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    sys_result_label_->setText(QString("错误上报已%1").arg(enable ? "使能" : "失能"));
    timer_->start();
}

void FullControlWindow::change_can_id() {
    timer_->stop();

    // 第一步：扫描总线上在线的 ID（逐 ID 扫描）
    std::vector<uint8_t> ids;
    if (!motor_can::enumerate_can_ids(comm_, ids)) {
        sys_result_label_->setText("未检测到设备（逐 ID 扫描无回复）");
        timer_->start();
        return;
    }

    // 目标 ID 不在线则拒绝：按地址写，写不存在的 ID 只会白等回显
    const uint8_t current_id = static_cast<uint8_t>(motor_id_);
    if (std::find(ids.begin(), ids.end(), current_id) == ids.end()) {
        QString list;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i != 0) {
                list += ", ";
            }
            list += QString::number(ids[i]);
        }
        sys_result_label_->setText(QString("当前 ID %1 不在线（在线：%2）。请确认电机当前 CAN ID。")
                                       .arg(current_id)
                                       .arg(list));
        timer_->start();
        return;
    }

    // 确认后才写（默认按钮选「否」，危险操作不强推）
    const int new_id = new_id_box_->value();
    if (!confirm(this, "修改 CAN ID",
                 QString("将把 ID %1 的电机改为 ID %2，写入后立即切到新 ID。\n\n"
                         "注意：\n"
                         "· 若有其他电机也在 ID %1，会被一起改；\n"
                         "· 新 ID 保存到 ROM，需重启该电机（断电→上电）才持久化。\n\n"
                         "确认修改？")
                     .arg(current_id)
                     .arg(new_id))) {
        sys_result_label_->setText("已取消");
        timer_->start();
        return;
    }

    if (!motor_can::write_can_id(comm_, current_id, static_cast<uint8_t>(new_id))) {
        sys_result_label_->setText("写 CAN ID 失败：无回显");
        timer_->start();
        return;
    }

    // 成功：电机已切到新 ID，把选择框切过去 → 触发 valueChanged → rebuild_motor 换句柄
    id_spin_->setValue(new_id);
    sys_result_label_->setText(
        QString("已写入 CAN ID %1（电机已切到新 ID）。请重启该电机（断电→上电）以持久化。")
            .arg(new_id));
    timer_->start();
}

void FullControlWindow::send_active_report() {
    timer_->stop();
    const uint8_t report_cmd = static_cast<uint8_t>(report_cmd_box_->currentData().toUInt());
    const bool enable = report_enable_check_->isChecked();
    const uint16_t interval = static_cast<uint16_t>(report_interval_box_->value());
    if (!motor_->set_active_report(report_cmd, enable, interval)) {
        sys_result_label_->setText("发送失败（总线未打开）");
        timer_->start();
        return;
    }
    sys_result_label_->setText(QString("已发送 0xB6：指令 0x%1 %2，间隔 %3×10ms")
                                   .arg(report_cmd, 2, 16, QLatin1Char('0'))
                                   .arg(enable ? "使能" : "关闭")
                                   .arg(interval));
    timer_->start();
}

void FullControlWindow::write_protect_time() {
    timer_->stop();
    const uint32_t ms = static_cast<uint32_t>(protect_time_box_->value());
    if (!confirm(this, "通讯中断保护",
                 QString("设置通讯中断保护时间 %1 ms（写 ROM，0=关闭）。\n\n"
                         "⚠️ 通讯中断超时后会自动切断输出并锁死抱闸；\n"
                         "避免在电机刚启动或运动时写入。确认？")
                     .arg(ms))) {
        timer_->start();
        return;
    }
    if (!motor_->set_com_protect_time(ms)) {
        sys_result_label_->setText("写入失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    sys_result_label_->setText(QString("已设置中断保护 %1 ms").arg(ms));
    timer_->start();
}

void FullControlWindow::write_position_limits() {
    timer_->stop();
    const double max_pos = limit_pos_box_->value();
    const double max_neg = limit_neg_box_->value();
    if (!confirm(this, "位置运行限位",
                 QString("设置位置运行最大正角度 %1 °、最大负角度 %2 °（写 ROM 立即生效）。\n\n"
                         "⚠️ 限位一旦写死，位置指令超出会被钳制；单位按 0.01°/LSB 打包，"
                         "手册未注明、真机待确认。确认？")
                     .arg(max_pos, 0, 'f', 2)
                     .arg(max_neg, 0, 'f', 2))) {
        timer_->start();
        return;
    }
    if (!motor_->set_position_limits(max_pos, max_neg)) {
        sys_result_label_->setText("写入失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    sys_result_label_->setText(QString("已设置位置限位 +%1 ° / %2 °").arg(max_pos, 0, 'f', 2)
                                   .arg(max_neg, 0, 'f', 2));
    timer_->start();
}

void FullControlWindow::reset_motor() {
    timer_->stop();
    if (!confirm(this, "系统复位", "发送 0x76 系统复位？\n\n"
                                   "· 0x64 新零点、0x20 相关配置在复位后生效；\n"
                                   "· 复位无回复，电机将重启。确认？")) {
        timer_->start();
        return;
    }
    // 0x76 无回复，fire-and-forget
    comm_.send(motor_can::encode_command(motor_id_, motor_can::RhCmd::SystemReset));
    sys_result_label_->setText("已发送系统复位（0x76）");
    timer_->start();
}

// ---------------------------------------------------------------------------
// Tab6 多机
// ---------------------------------------------------------------------------

QWidget* FullControlWindow::build_tab_multi() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    // 顺序发送 0xA4
    seq_angles_edit_ = new QLineEdit(page);
    seq_angles_edit_->setPlaceholderText("如 180,360,720 → 电机 1,2,3");
    seq_speed_box_ = make_spin(page, 1, 3000, 30, " °/s");
    auto* seq_send_btn = new QPushButton("按顺序发送", page);

    auto* seq_box = new QGroupBox("多电机顺序 (0xA4)", page);
    auto* sf = new QFormLayout(seq_box);
    sf->addRow("角度列表 (°)", seq_angles_edit_);
    sf->addRow("转速", row({seq_speed_box_}, seq_send_btn));
    auto* seq_note = new QLabel(
        "逗号分隔的角度，按位置对应电机 ID：第 1 个 → 电机 1，第 2 个 → 电机 2……\n"
        "逐个发送并等待每台电机回复。", page);
    seq_note->setWordWrap(true);
    sf->addRow(seq_note);
    layout->addWidget(seq_box);

    connect(seq_send_btn, &QPushButton::clicked, this, [this] { send_sequence(); });

    // 0x280 广播
    bcast_angle_box_ = make_double(page, -1.0e6, 1.0e6, 2, " °");
    bcast_speed_box_ = make_spin(page, 1, 3000, 30, " °/s");
    auto* bcast_stop_btn = new QPushButton("广播停止", page);
    auto* bcast_off_btn = new QPushButton("广播关闭", page);
    auto* bcast_pos_btn = new QPushButton("广播位置", page);

    auto* bcast_box = new QGroupBox("0x280 广播", page);
    auto* bf = new QFormLayout(bcast_box);
    bf->addRow("位置目标/转速", row({bcast_angle_box_, bcast_speed_box_}));
    bf->addRow("操作", row({bcast_stop_btn, bcast_off_btn, bcast_pos_btn}));
    auto* bcast_note = new QLabel(
        "所有电机同时响应，各自在 0x240+ID 回复。前置：先到「系统」tab 失能 CANID 滤波器"
        "（0x20/0x02），否则广播可能被滤掉。", page);
    bcast_note->setWordWrap(true);
    bf->addRow(bcast_note);
    layout->addWidget(bcast_box);

    connect(bcast_stop_btn, &QPushButton::clicked, this, [this] { broadcast_stop(); });
    connect(bcast_off_btn, &QPushButton::clicked, this, [this] { broadcast_off(); });
    connect(bcast_pos_btn, &QPushButton::clicked, this, [this] { broadcast_position(); });

    multi_result_label_ = new QLabel("--", page);
    multi_result_label_->setWordWrap(true);
    layout->addWidget(multi_result_label_);

    layout->addStretch(1);
    return page;
}

void FullControlWindow::send_sequence() {
    // 解析逗号分隔的角度列表，非数字即报错
    std::vector<double> angles;
    const QStringList parts = seq_angles_edit_->text().split(',', Qt::SkipEmptyParts);
    for (const QString& p : parts) {
        bool ok = false;
        const double a = p.trimmed().toDouble(&ok);
        if (!ok) {
            multi_result_label_->setText(QString("角度列表无效：'%1' 不是数字").arg(p.trimmed()));
            return;
        }
        angles.push_back(a);
    }
    if (angles.empty()) {
        multi_result_label_->setText("角度列表为空，如 180,360,720");
        return;
    }
    if (angles.size() > 32) {
        multi_result_label_->setText("角度数超过 32，无法映射到电机 ID（最多 32 台）");
        return;
    }

    timer_->stop();
    const uint16_t speed = static_cast<uint16_t>(seq_speed_box_->value());
    QStringList results;
    for (size_t i = 0; i < angles.size(); ++i) {
        const uint8_t id = static_cast<uint8_t>(i + 1);
        const motor_can::CanFrame frame = motor_can::encode_position(id, angles[i], speed);
        motor_can::CanFrame reply;
        motor_can::MotorRunStatus st;
        const bool ok = comm_.send(frame) &&
                        comm_.receive_by_id(0x240u + id, reply, std::chrono::milliseconds(100)) &&
                        motor_can::decode_run_status(reply, st);
        results << QString("ID %1 → %2°：%3")
                       .arg(id)
                       .arg(angles[i], 0, 'f', 2)
                       .arg(ok ? "已发送" : "无回复");
    }
    multi_result_label_->setText(results.join("\n"));
    timer_->start();
}

void FullControlWindow::broadcast_stop() {
    // 紧急停通常不需要确认框，立即广播
    comm_.send(motor_can::to_broadcast(
        motor_can::encode_command(1, motor_can::RhCmd::MotorStop)));
    multi_result_label_->setText("已广播 0x81 停止（0x280）");
}

void FullControlWindow::broadcast_off() {
    if (!confirm(this, "广播关闭", "广播 0x80 关闭所有电机输出（清除运行状态）？")) {
        return;
    }
    comm_.send(motor_can::to_broadcast(
        motor_can::encode_command(1, motor_can::RhCmd::MotorOff)));
    multi_result_label_->setText("已广播 0x80 关闭（0x280）");
}

void FullControlWindow::broadcast_position() {
    const double angle = bcast_angle_box_->value();
    const uint16_t speed = static_cast<uint16_t>(bcast_speed_box_->value());
    if (!confirm(this, "广播位置",
                 QString("广播 0xA4：所有电机同时运动到 %1 °（限速 %2 °/s）？")
                     .arg(angle, 0, 'f', 2)
                     .arg(speed))) {
        return;
    }
    comm_.send(motor_can::to_broadcast(motor_can::encode_position(1, angle, speed)));
    multi_result_label_->setText(QString("已广播 0xA4 到 %1 °（0x280）").arg(angle, 0, 'f', 2));
}

void FullControlWindow::scan_online() {
    timer_->stop();
    std::vector<uint8_t> ids;
    if (!motor_can::enumerate_can_ids(comm_, ids)) {
        online_label_->setText("未检测到设备");
        timer_->start();
        return;
    }
    QString list;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i != 0) {
            list += ", ";
        }
        list += QString::number(ids[i]);
    }
    online_label_->setText(QString("在线：%1").arg(list));
    timer_->start();
}

// ---------------------------------------------------------------------------
// Tab7 波形
// ---------------------------------------------------------------------------

QWidget* FullControlWindow::build_tab_waveform() {
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);

    // 四路波形控件拉伸占满 tab 剩余空间；数据由 poll() 每 300ms 喂入
    waveform_ = new WaveformView(kWaveformMaxPoints, kWaveformIntervalS, page);
    layout->addWidget(waveform_, 1);

    // 暂停/保存/清空都是界面操作、不影响电机，不停止轮询、也不弹确认框
    auto* pause_btn = new QPushButton("暂停", page);
    auto* save_btn = new QPushButton("保存图片", page);
    auto* clear_btn = new QPushButton("清空", page);
    layout->addWidget(row({pause_btn, save_btn, clear_btn}));

    auto* note = new QLabel(
        "电压取 0x9A，转速/角度/电流取 0x9C，随轮询每 300ms 记录一点，满窗 60s 自动滚动。\n"
        "「暂停」冻结采样、「清空」只清曲线（角度累计基准保留）、「保存图片」存当前画面为 PNG。",
        page);
    note->setWordWrap(true);
    layout->addWidget(note);

    connect(pause_btn, &QPushButton::clicked, this, [this, pause_btn] {
        const bool paused = !waveform_->is_paused();
        waveform_->set_paused(paused);
        pause_btn->setText(paused ? "继续" : "暂停");
    });
    connect(save_btn, &QPushButton::clicked, this, [this] { save_waveform_image(); });
    connect(clear_btn, &QPushButton::clicked, this,
            [this] { waveform_->clear_samples(); });

    return page;
}

void FullControlWindow::save_waveform_image() {
    // 默认存到「图片」目录（无桌面环境则退回家目录），文件名带电机 ID 和时间戳
    QString dir = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    if (dir.isEmpty()) {
        dir = QDir::homePath();
    }
    const QString default_name =
        QString("waveform_id%1_%2.png")
            .arg(motor_id_)
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    const QString path = QFileDialog::getSaveFileName(this, "保存波形图片",
                                                      dir + "/" + default_name,
                                                      "PNG 图片 (*.png)");
    if (path.isEmpty()) {
        return;  // 用户取消
    }
    if (!waveform_->save_snapshot(path)) {
        status_label_->setText("波形保存失败");
        return;
    }
    status_label_->setText(QString("波形已保存：%1").arg(path));
}
