// gui/motor_monitor/monitor_window.cpp
// 电机监测窗口实现：
//  - QTimer 主线程轮询所选电机的 0x9A 状态 + 0x9C 运行数据并刷新标签（只读）。
//  - 三环 PID：读取（0x30）填框；应用（0x31 RAM）/ 保存（0x32 ROM）批量写可配置环。
//  - 多圈零点：记录当前编码器位置为多圈零点（0x64），需 0x76 复位后生效。
//  - 全部收发走 Device 层 Motor 的过滤接口（杂帧按命令字节丢弃），可与另一控制
//    程序同时运行；PID / 零点操作执行期间暂停轮询 timer，避免与自身轮询串扰。

#include "monitor_window.hpp"

#include "motor_can/can_comm/can_comm.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace {

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

// PID 字段：环索引 + 对应输入框
struct PidField {
    motor_can::PidIndex index;
    QDoubleSpinBox* box;
};

}  // namespace

MonitorWindow::MonitorWindow(motor_can::CanComm& comm) : comm_(comm) {
    setWindowTitle("电机状态监测");
    setMinimumWidth(380);

    // 只读句柄：不归0（home_on_init=false），回复等待 100ms 保证界面灵敏
    mcfg_.home_on_init = false;
    mcfg_.reply_timeout = std::chrono::milliseconds(100);
    rebuild_motor();

    id_spin_ = new QSpinBox(this);
    id_spin_->setRange(1, 32);
    id_spin_->setValue(motor_id_);

    status_label_ = new QLabel("--", this);
    temp_label_ = new QLabel("--", this);
    mos_label_ = new QLabel("--", this);
    volt_label_ = new QLabel("--", this);
    brake_label_ = new QLabel("--", this);
    error_label_ = new QLabel("--", this);
    speed_label_ = new QLabel("--", this);
    current_label_ = new QLabel("--", this);
    angle_label_ = new QLabel("--", this);

    auto* layout = new QVBoxLayout(this);

    // 顶部：电机选择 + 状态
    auto* top = new QFormLayout;
    top->addRow("电机 ID (1~32)", id_spin_);
    top->addRow("状态", status_label_);
    layout->addLayout(top);

    // 0x9A 状态区
    auto* status_box = new QGroupBox("电机状态 (0x9A)", this);
    auto* sf = new QFormLayout(status_box);
    sf->addRow("电机温度", temp_label_);
    sf->addRow("MOS 温度", mos_label_);
    sf->addRow("电压 (V)", volt_label_);
    sf->addRow("抱闸", brake_label_);
    sf->addRow("错误标志", error_label_);
    layout->addWidget(status_box);

    // 0x9C 运行数据区
    auto* run_box = new QGroupBox("运行数据 (0x9C)", this);
    auto* rf = new QFormLayout(run_box);
    rf->addRow("转速 (dps)", speed_label_);
    rf->addRow("转矩电流 (A)", current_label_);
    rf->addRow("输出轴角度 (°)", angle_label_);
    layout->addWidget(run_box);

    // 三环 PID 区（电流环只读）
    cur_kp_box_ = make_pid_spin(this, false);
    cur_ki_box_ = make_pid_spin(this, false);
    spd_kp_box_ = make_pid_spin(this, true);
    spd_ki_box_ = make_pid_spin(this, true);
    pos_kp_box_ = make_pid_spin(this, true);
    pos_ki_box_ = make_pid_spin(this, true);
    pos_kd_box_ = make_pid_spin(this, true);

    auto* pid_box = new QGroupBox("三环 PID (0x30/0x31/0x32)", this);
    auto* pf = new QFormLayout(pid_box);
    pf->addRow("电流环 KP（只读）", cur_kp_box_);
    pf->addRow("电流环 KI（只读）", cur_ki_box_);
    pf->addRow("速度环 KP", spd_kp_box_);
    pf->addRow("速度环 KI", spd_ki_box_);
    pf->addRow("位置环 KP", pos_kp_box_);
    pf->addRow("位置环 KI", pos_ki_box_);
    pf->addRow("位置环 KD", pos_kd_box_);
    auto* read_btn = new QPushButton("读取", this);
    auto* ram_btn = new QPushButton("应用(RAM)", this);
    auto* rom_btn = new QPushButton("保存(ROM)", this);
    auto* btn_wrap = new QWidget(this);
    auto* bh = new QHBoxLayout(btn_wrap);
    bh->setContentsMargins(0, 0, 0, 0);
    bh->addWidget(read_btn);
    bh->addWidget(ram_btn);
    bh->addWidget(rom_btn);
    pf->addRow("操作", btn_wrap);
    layout->addWidget(pid_box);
    connect(read_btn, &QPushButton::clicked, this, [this] { read_pid_all(); });
    connect(ram_btn, &QPushButton::clicked, this,
            [this] { write_pid_all(motor_can::RhCmd::PidWriteRam); });
    connect(rom_btn, &QPushButton::clicked, this,
            [this] { write_pid_all(motor_can::RhCmd::PidWriteRom); });

    // 多圈零点区（0x64）
    auto* zero_box = new QGroupBox("多圈零点 (0x64)", this);
    auto* zf = new QVBoxLayout(zero_box);
    auto* zero_note = new QLabel(
        "将当前编码器位置记录为多圈零点并写入 ROM。\n"
        "需发送 0x76 系统复位后新零点才生效；避免电机刚启动/运动时写入。", this);
    zero_note->setWordWrap(true);
    auto* zero_btn = new QPushButton("记录当前点为零点", this);
    zero_result_label_ = new QLabel("--", this);
    zf->addWidget(zero_note);
    zf->addWidget(zero_btn);
    zf->addWidget(zero_result_label_);
    layout->addWidget(zero_box);
    connect(zero_btn, &QPushButton::clicked, this, [this] { set_zero_point(); });

    // 修改 CAN ID 区（0x20 索引 0x05，按地址写；发送前先扫描总线上在线的 ID）
    new_id_spin_ = new QSpinBox(this);
    new_id_spin_->setRange(1, 32);
    new_id_spin_->setValue(motor_id_);
    change_id_btn_ = new QPushButton("修改", this);
    auto* canid_box = new QGroupBox("修改 CAN ID (0x20)", this);
    auto* cf = new QFormLayout(canid_box);
    cf->addRow("新 CAN ID (1~32)", new_id_spin_);
    auto* cbtn_wrap = new QWidget(this);
    auto* cbh = new QHBoxLayout(cbtn_wrap);
    cbh->setContentsMargins(0, 0, 0, 0);
    cbh->addWidget(change_id_btn_);
    cf->addRow("操作", cbtn_wrap);
    auto* canid_note = new QLabel(
        "按地址写入：只改当前 ID 的电机（同 ID 的多台会一起被改）。\n"
        "写入后立即切到新 ID，但需重启该电机（断电→上电）才持久化。", this);
    canid_note->setWordWrap(true);
    canid_result_label_ = new QLabel("--", this);
    cf->addRow(canid_note);
    cf->addRow(canid_result_label_);
    layout->addWidget(canid_box);
    connect(change_id_btn_, &QPushButton::clicked, this, [this] { change_can_id(); });

    // ID 切换立即生效：重建只读 Motor 句柄（只换对象，不驱动电机）
    connect(id_spin_, qOverload<int>(&QSpinBox::valueChanged),
            this, [this](int v) {
                motor_id_ = v;
                rebuild_motor();
            });

    // 每 300ms 轮询一次状态 + 运行数据
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this] { poll(); });
    timer_->start(300);
}

MonitorWindow::~MonitorWindow() = default;

void MonitorWindow::rebuild_motor() {
    motor_ = std::make_unique<motor_can::Motor>(comm_, static_cast<uint8_t>(motor_id_), mcfg_);
}

void MonitorWindow::poll() {
    // 0x9A 状态：过滤读（杂帧丢弃），Motor 内部 100ms 内等命令字节匹配
    motor_can::MotorStatus st;
    if (!motor_->read_status(st)) {
        status_label_->setText(QString("电机 ID %1 无响应").arg(motor_id_));
        return;
    }

    temp_label_->setText(QString("%1 ℃").arg(st.temp_c));
    mos_label_->setText(QString("%1 ℃").arg(st.mos_temp_c));
    volt_label_->setText(QString("%1 V").arg(st.voltage_v, 0, 'f', 1));
    brake_label_->setText(st.brake_released ? "已释放" : "锁死");
    error_label_->setText(st.error_state ? QString("0x%1").arg(st.error_state, 4, 16, QLatin1Char('0'))
                                         : QString("0x0000（无错误）"));
    status_label_->setText(QString("电机 ID %1 在线").arg(motor_id_));

    // 0x9C 运行数据
    motor_can::MotorRunStatus rs;
    if (!motor_->read_run_status(rs)) {
        // 0x9A 有响应而 0x9C 无响应：保留上一次运行数据，仅标黄状态栏
        status_label_->setText(QString("电机 ID %1 在线（0x9C 无响应）").arg(motor_id_));
        return;
    }
    speed_label_->setText(QString("%1 dps").arg(rs.speed_dps, 0, 'f', 0));
    current_label_->setText(QString("%1 A").arg(rs.iq_a, 0, 'f', 2));
    angle_label_->setText(QString("%1 °").arg(rs.angle_deg, 0, 'f', 1));
}

void MonitorWindow::read_pid_all() {
    // 7 个环参数（含电流环，仅展示）
    const PidField fields[] = {
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
        if (!motor_->read_pid(f.index, value)) {
            ok = false;
            status_label_->setText(QString("读取 PID(索引 0x%1) 失败")
                                       .arg(static_cast<int>(f.index), 2, 16, QLatin1Char('0')));
            break;
        }
        f.box->setValue(value);
    }
    if (ok) {
        status_label_->setText("PID 已读取");
    }
    timer_->start();
}

void MonitorWindow::write_pid_all(motor_can::RhCmd cmd) {
    // 只写可配置环（速度 Kp/Ki、位置 Kp/Ki/Kd），电流环只读不写
    const PidField fields[] = {
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
                              ? motor_->write_pid_ram(f.index, f.box->value())
                              : motor_->write_pid_rom(f.index, f.box->value());
        if (!done) {
            ok = false;
            status_label_->setText(QString("写 PID(索引 0x%1) 失败")
                                       .arg(static_cast<int>(f.index), 2, 16, QLatin1Char('0')));
            break;
        }
    }
    if (ok) {
        status_label_->setText(cmd == motor_can::RhCmd::PidWriteRam ? "PID 已写入 RAM（掉电丢失）"
                                                                    : "PID 已写入 ROM（掉电保存）");
    }
    timer_->start();
}

void MonitorWindow::set_zero_point() {
    const auto choice = QMessageBox::question(
        this, "记录零点",
        "将当前编码器位置记录为多圈零点并写入 ROM？\n\n"
        "注意：需发送 0x76 系统复位后新零点才生效；避免电机运动时写入。",
        QMessageBox::Yes | QMessageBox::No);
    if (choice != QMessageBox::Yes) {
        return;
    }
    int32_t offset = 0;
    if (!motor_->set_zero_point(offset)) {
        zero_result_label_->setText("记录失败：无回复或命令字节不符");
        return;
    }
    zero_result_label_->setText(QString("已记录，新零偏 = %1 脉冲（0x76 复位后生效）").arg(offset));
    QMessageBox::information(this, "记录零点",
                             QString("已记录当前点为零点（新零偏 %1 脉冲）。\n"
                                     "需发送 0x76 系统复位后新零点生效。")
                                 .arg(offset));
}

void MonitorWindow::change_can_id() {
    timer_->stop();

    // 第一步：扫描总线上在线的 ID（0x79 广播读在 X2-7 上无效，改逐 ID 扫描）
    std::vector<uint8_t> ids;
    if (!motor_can::enumerate_can_ids(comm_, ids)) {
        canid_result_label_->setText("未检测到设备（逐 ID 扫描无回复）");
        timer_->start();
        return;
    }

    // 目标 ID 不在线则拒绝：按地址写，写一台不存在的 ID 只会白等回显
    const uint8_t current_id = static_cast<uint8_t>(motor_id_);
    if (std::find(ids.begin(), ids.end(), current_id) == ids.end()) {
        QString list;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i != 0) list += ", ";
            list += QString::number(ids[i]);
        }
        canid_result_label_->setText(
            QString("当前 ID %1 不在线（在线：%2）。请确认电机当前 CAN ID。")
                .arg(current_id)
                .arg(list));
        timer_->start();
        return;
    }

    // 确认后才写（默认按钮选「否」，危险操作不强推）
    const int new_id = new_id_spin_->value();
    const auto choice = QMessageBox::question(
        this, "修改 CAN ID",
        QString("将把 ID %1 的电机改为 ID %2，写入后立即切到新 ID。\n\n"
                "注意：\n"
                "· 若有其他电机也在 ID %1，会被一起改；\n"
                "· 新 ID 保存到 ROM，需重启该电机（断电→上电）才持久化。\n\n"
                "确认修改？")
            .arg(current_id)
            .arg(new_id),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (choice != QMessageBox::Yes) {
        canid_result_label_->setText("已取消");
        timer_->start();
        return;
    }

    if (!motor_can::write_can_id(comm_, current_id, static_cast<uint8_t>(new_id))) {
        canid_result_label_->setText("写 CAN ID 失败：无回显");
        timer_->start();
        return;
    }

    // 成功：电机已切到新 ID，把选择框切过去 → 触发 valueChanged → rebuild_motor 换句柄
    id_spin_->setValue(new_id);
    canid_result_label_->setText(
        QString("已写入 CAN ID %1（电机已切到新 ID）。\n"
                "请重启该电机（断电→上电）以持久化。")
            .arg(new_id));
    timer_->start();
}
