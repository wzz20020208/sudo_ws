// gui/motor_position_control/position_window.cpp
// 位置闭环控制窗口实现：
//  - QTimer 主线程轮询所选电机的 0x92 当前角度 + 0x9C 运行数据并刷新实时区。
//  - 运行到目标 / 回0点：发 0xA4 绝对位置指令（带最大转速）。
//  - 抱闸手动：开闸 0x77；锁闸先 0x81 停止再 0x78。
//  - 控制动作串行收发，执行期间暂停轮询 timer，防止杂帧混入接收队列。

#include "position_window.hpp"

#include "motor_can/can_comm/can_comm.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <vector>

namespace {

// 命令字节在 Data[0]，其余为 0
constexpr uint8_t kAngleCmd = 0x92;  // 读多圈绝对角度
constexpr uint8_t kRunCmd = 0x9C;    // 运行数据：温度/转矩电流/转速/角度（只读）

// 单字节命令回复为原帧回显，仅校验命令字节一致
bool echo_ok(const motor_can::CanFrame& reply, motor_can::RhCmd cmd) {
    return reply.data[0] == static_cast<uint8_t>(cmd);
}

}  // namespace

PositionWindow::PositionWindow(motor_can::CanComm& comm) : comm_(comm) {
    setWindowTitle("位置闭环控制");
    setMinimumWidth(360);

    id_spin_ = new QSpinBox(this);
    id_spin_->setRange(1, 32);
    id_spin_->setValue(motor_id_);

    status_label_ = new QLabel("--", this);
    target_box_ = new QDoubleSpinBox(this);
    target_box_->setRange(-1.0e6, 1.0e6);
    target_box_->setDecimals(2);
    speed_box_ = new QSpinBox(this);
    speed_box_->setRange(1, 3000);
    speed_box_->setValue(10);
    speed_box_->setSuffix(" dps");

    angle_label_ = new QLabel("--", this);
    speed_label_ = new QLabel("--", this);
    current_label_ = new QLabel("--", this);

    auto* layout = new QVBoxLayout(this);

    // 顶部：电机选择 + 状态
    auto* top = new QFormLayout;
    top->addRow("电机 ID (1~32)", id_spin_);
    top->addRow("状态", status_label_);
    layout->addLayout(top);

    // 目标位置区
    auto* target_box = new QGroupBox("目标位置 (0xA4)", this);
    auto* tf = new QFormLayout(target_box);
    tf->addRow("目标角度 (°，多圈)", target_box_);
    tf->addRow("最大转速", speed_box_);

    auto* release_btn = new QPushButton("开闸", this);
    auto* run_btn = new QPushButton("运行到目标", this);
    auto* home_btn = new QPushButton("回0点", this);
    auto* stop_btn = new QPushButton("停止", this);
    auto* lock_btn = new QPushButton("锁闸", this);

    auto* move_row = new QWidget(this);
    auto* move_layout = new QHBoxLayout(move_row);
    move_layout->setContentsMargins(0, 0, 0, 0);
    move_layout->addWidget(run_btn);
    move_layout->addWidget(home_btn);

    auto* brake_row = new QWidget(this);
    auto* brake_layout = new QHBoxLayout(brake_row);
    brake_layout->setContentsMargins(0, 0, 0, 0);
    brake_layout->addWidget(release_btn);
    brake_layout->addWidget(stop_btn);
    brake_layout->addWidget(lock_btn);

    tf->addRow("运动", move_row);
    tf->addRow("抱闸/停止", brake_row);
    layout->addWidget(target_box);

    // 实时区
    auto* run_box = new QGroupBox("实时数据", this);
    auto* rf = new QFormLayout(run_box);
    rf->addRow("当前角度 (°)", angle_label_);
    rf->addRow("转速 (dps)", speed_label_);
    rf->addRow("转矩电流 (A)", current_label_);
    layout->addWidget(run_box);

    // 多电机顺序：逗号分隔角度列表 → 第 i 个角度发到 ID i+1
    seq_angles_edit_ = new QLineEdit(this);
    seq_angles_edit_->setPlaceholderText("如 180,360,720 → 电机 1,2,3");
    seq_send_btn_ = new QPushButton("按顺序发送", this);
    seq_result_label_ = new QLabel("--", this);
    seq_result_label_->setWordWrap(true);

    auto* seq_box = new QGroupBox("多电机顺序 (0xA4)", this);
    auto* sf = new QFormLayout(seq_box);
    sf->addRow("角度列表 (°)", seq_angles_edit_);
    auto* seq_btn_wrap = new QWidget(this);
    auto* seq_btn_layout = new QHBoxLayout(seq_btn_wrap);
    seq_btn_layout->setContentsMargins(0, 0, 0, 0);
    seq_btn_layout->addWidget(seq_send_btn_);
    sf->addRow("操作", seq_btn_wrap);
    auto* seq_note = new QLabel(
        "逗号分隔的角度，按位置对应电机 ID：第 1 个 → 电机 1，第 2 个 → 电机 2……\n"
        "转速用上方「最大转速」。逐个发送并等待每台电机回复。", this);
    seq_note->setWordWrap(true);
    sf->addRow(seq_note);
    sf->addRow(seq_result_label_);
    layout->addWidget(seq_box);

    connect(seq_send_btn_, &QPushButton::clicked, this, [this] { send_sequence(); });

    // ID 切换立即生效
    connect(id_spin_, qOverload<int>(&QSpinBox::valueChanged),
            this, [this](int v) { motor_id_ = v; });

    connect(release_btn, &QPushButton::clicked, this, [this] { brake_release(); });
    connect(run_btn, &QPushButton::clicked, this, [this] { run_to(target_box_->value()); });
    connect(home_btn, &QPushButton::clicked, this, [this] { run_to(0.0); });
    connect(stop_btn, &QPushButton::clicked, this, [this] { motor_stop(); });
    connect(lock_btn, &QPushButton::clicked, this, [this] { brake_lock(); });

    // 每 300ms 轮询当前角度 + 运行数据
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this] { poll(); });
    timer_->start(300);
}

PositionWindow::~PositionWindow() = default;

bool PositionWindow::query(uint8_t cmd, motor_can::CanFrame& reply) {
    uint8_t data[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};
    motor_can::CanFrame f;
    f.id = 0x140u + motor_id_;
    f.is_extended = false;
    f.dlc = 8;
    std::copy_n(data, 8, f.data);

    return comm_.send(f) &&
           comm_.receive_by_id(0x240u + motor_id_, reply, std::chrono::milliseconds(100));
}

bool PositionWindow::send_and_receive(const motor_can::CanFrame& frame,
                                      motor_can::CanFrame& reply) {
    return comm_.send(frame) &&
           comm_.receive_by_id(0x240u + motor_id_, reply, std::chrono::milliseconds(100));
}

void PositionWindow::poll() {
    motor_can::CanFrame reply;

    // 0x92 当前多圈角度：Data[4..7] int32（0.01°/LSB）
    double angle = 0.0;
    if (!query(kAngleCmd, reply) || !motor_can::decode_angle(reply, angle)) {
        status_label_->setText(QString("电机 ID %1 无响应").arg(motor_id_));
        return;
    }
    angle_label_->setText(QString("%1 °").arg(angle, 0, 'f', 2));
    status_label_->setText(QString("电机 ID %1 在线").arg(motor_id_));

    // 0x9C 运行数据
    if (!query(kRunCmd, reply)) {
        return;
    }
    motor_can::MotorRunStatus st;
    if (motor_can::decode_run_status(reply, st)) {
        speed_label_->setText(QString("%1 dps").arg(st.speed_dps, 0, 'f', 1));
        current_label_->setText(QString("%1 A").arg(st.iq_a, 0, 'f', 2));
    }
}

void PositionWindow::run_to(double angle_deg) {
    timer_->stop();
    const motor_can::CanFrame frame = motor_can::encode_position(
        motor_id_, angle_deg, static_cast<uint16_t>(speed_box_->value()));
    motor_can::CanFrame reply;
    motor_can::MotorRunStatus st;
    if (!send_and_receive(frame, reply) || !motor_can::decode_run_status(reply, st)) {
        status_label_->setText("运行失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText(angle_deg == 0.0
                               ? "回0点指令已发送"
                               : QString("已发送到 %1 °").arg(angle_deg, 0, 'f', 2));
    timer_->start();
}

void PositionWindow::brake_release() {
    timer_->stop();
    motor_can::CanFrame reply;
    if (!send_and_receive(motor_can::encode_command(motor_id_, motor_can::RhCmd::BrakeRelease),
                          reply) ||
        !echo_ok(reply, motor_can::RhCmd::BrakeRelease)) {
        status_label_->setText("开闸失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText("抱闸已释放");
    timer_->start();
}

void PositionWindow::motor_stop() {
    timer_->stop();
    motor_can::CanFrame reply;
    if (!send_and_receive(motor_can::encode_command(motor_id_, motor_can::RhCmd::MotorStop), reply) ||
        !echo_ok(reply, motor_can::RhCmd::MotorStop)) {
        status_label_->setText("停止失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText("电机已停止");
    timer_->start();
}

void PositionWindow::brake_lock() {
    timer_->stop();
    // 先停再锁，避免带转锁闸
    motor_can::CanFrame reply;
    if (!send_and_receive(motor_can::encode_command(motor_id_, motor_can::RhCmd::MotorStop), reply) ||
        !echo_ok(reply, motor_can::RhCmd::MotorStop)) {
        status_label_->setText("锁闸前停止失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    if (!send_and_receive(motor_can::encode_command(motor_id_, motor_can::RhCmd::BrakeLock), reply) ||
        !echo_ok(reply, motor_can::RhCmd::BrakeLock)) {
        status_label_->setText("锁闸失败：无回复或命令字节不符");
        timer_->start();
        return;
    }
    status_label_->setText("已停止并锁闸");
    timer_->start();
}

void PositionWindow::send_sequence() {
    // 解析逗号分隔的角度列表，非数字即报错
    std::vector<double> angles;
    const QStringList parts = seq_angles_edit_->text().split(',', Qt::SkipEmptyParts);
    for (const QString& p : parts) {
        bool ok = false;
        const double a = p.trimmed().toDouble(&ok);
        if (!ok) {
            seq_result_label_->setText(QString("角度列表无效：'%1' 不是数字").arg(p.trimmed()));
            return;
        }
        angles.push_back(a);
    }
    if (angles.empty()) {
        seq_result_label_->setText("角度列表为空，如 180,360,720");
        return;
    }
    if (angles.size() > 32) {
        seq_result_label_->setText("角度数超过 32，无法映射到电机 ID（最多 32 台）");
        return;
    }

    timer_->stop();
    const uint16_t speed = static_cast<uint16_t>(speed_box_->value());
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
    seq_result_label_->setText(results.join("\n"));
    timer_->start();
}
