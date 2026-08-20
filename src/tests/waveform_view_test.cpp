// tests/waveform_view_test.cpp
// WaveformView 离线测试（无硬件）：
//  - 生成两份 7 列 CSV（新格式含 t_epoch 现实时间戳）：A 从 epoch T0 起 10s，
//    B 从 epoch T0+5s 起 8s，模拟主/从两路分别录制；
//  - 验证 add_csv 解析/角度解包/回放不裁剪、按现实时间戳对齐叠加（B 首样本 t_s≈5）、
//    连缩 50 级不崩、回实时后喂数恢复、存 PNG。
// 运行: ./build/waveform_view_test

#include "gui/widgets/waveform_view.hpp"

#include <QApplication>
#include <QByteArray>
#include <QFile>
#include <QTemporaryDir>
#include <QtGlobal>

// Release 默认带 -DNDEBUG 会把 assert 编译掉，这里先取消 NDEBUG 让断言真正生效
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>

// 把累计角编码成 0x9C 的原始 int16（±32768 模回绕，1°/LSB）
static double wrap_int16(double cumulative_deg) {
    double raw = std::fmod(cumulative_deg + 32768.0, 65536.0);
    if (raw < 0.0) {
        raw += 65536.0;
    }
    return raw - 32768.0;
}

// 写新格式 7 列 CSV：motor_id,t_s,t_epoch,voltage_v,speed_dps,angle_deg,iq_a
// rows 行数（t_s 从 0 每 dt_s 一行），epoch 为第一行的现实时间戳起点
static void write_csv(const QString& path, double epoch, int rows, double dt_s,
                      double cumulative_start_deg) {
    std::ofstream out(path.toStdString());
    // 与真实录制端一致：fixed + 3 位小数（epoch 列 1.7e9 量级，默认格式会退成 3 位有效数字）
    out << std::fixed << std::setprecision(3);
    out << "motor_id,t_s,t_epoch,voltage_v,speed_dps,angle_deg,iq_a\n";
    for (int i = 0; i < rows; ++i) {
        const double t = dt_s * i;
        const double cumulative = cumulative_start_deg + 10.0 * i;
        out << 1 << ',' << t << ',' << (epoch + t) << ',' << 24.0 << ','
            << (i % 20) << ',' << wrap_int16(cumulative) << ",0.5\n";
    }
}

int main(int argc, char** argv) {
    // 无桌面环境（无 DISPLAY）时用离屏渲染，测试照样能跑
    if (qEnvironmentVariableIsEmpty("DISPLAY") && qEnvironmentVariableIsEmpty("WAYLAND_DISPLAY")) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
    QApplication app(argc, argv);

    QTemporaryDir dir;
    const QString path_a = dir.filePath("master.csv");
    const QString path_b = dir.filePath("slave.csv");
    // A：epoch T0 起 10s、1001 行 @10ms，累计角 30000→40000°（跨 int16 回绕点）
    // B：epoch T0+5.0s 起 8s、801 行 @10ms —— 与 A 按现实时间有 5s 错位
    const double kEpochA = 1.7e9;
    write_csv(path_a, kEpochA, 1001, 0.01, 30000.0);
    write_csv(path_b, kEpochA + 5.0, 801, 0.01, 50000.0);

    WaveformView w(200, 0.3);
    w.resize(800, 600);
    w.show();

    QString err;
    assert(w.add_csv(path_a, &err));              // 加载 A 成功
    assert(w.is_replay());                        // 进入回放模式
    assert(w.series_count() == 1);                // 单系列
    assert(w.samples().size() == 1001);           // 回放不裁剪（满 200 点上限之外也保留）
    assert(std::fabs(w.samples().back().angle_deg - 40000.0) < 1e-6);  // 解包成功、累计单调
    assert(std::fabs(w.samples().front().t_s - 0.0) < 1e-3);  // A 首样本 = 同步基准 0

    // 缩放应真实改变画面（防方向/钳位错误导致放大无效）
    const QString png_full = dir.filePath("full.png");
    const QString png_zoom = dir.filePath("zoom.png");
    assert(w.save_snapshot(png_full));
    for (int i = 0; i < 50; ++i) {
        w.zoom_in();                              // 缩到最小 10ms，验证不除零不崩
    }
    assert(w.save_snapshot(png_zoom));
    QFile f_full(png_full), f_zoom(png_zoom);
    assert(f_full.open(QIODevice::ReadOnly) && f_zoom.open(QIODevice::ReadOnly));
    assert(f_full.readAll() != f_zoom.readAll()); // 放大后画面应与满窗不同
    for (int i = 0; i < 50; ++i) {
        w.zoom_out();
    }

    // 叠加 B：按现实时间戳对齐，B 首样本应落在 t_s≈5s（相对 A 的基准），末样本 ≈13s
    assert(w.add_csv(path_b, &err));              // 加载 B 成功
    assert(w.series_count() == 2);                // 两系列叠加
    const auto& b = w.series_samples(1);
    assert(std::fabs(b.front().t_s - 5.0) < 1e-3);
    assert(std::fabs(b.back().t_s - 13.0) < 1e-3);
    assert(std::fabs(w.series_samples(0).front().t_s - 0.0) < 1e-3);  // A 首样本基准不动

    w.back_to_live();                            // 回到实时
    assert(!w.is_replay());
    assert(w.series_count() == 0);               // 叠加系列已清空
    w.append_sample(24.0, 10.0, 5.0, 0.1);       // 实时喂数恢复（不再丢弃）
    assert(w.series_count() == 1);               // 重建实时系列

    const QString png = dir.filePath("snap.png");
    assert(w.save_snapshot(png));                // 离屏 PNG 供人工目检
    return 0;
}
