// gui/widgets/angle_unwrap.hpp
// 输出轴角度 int16 回绕（±32767°）解包为累计角的公共函数。
// GUI 实时喂数（full_control_window::poll）与 CSV 历史加载（waveform_view::load_csv）
// 共用同一份解包逻辑；相邻采样间隔 ≤ 数百 ms 时阈值（半圈=±32768°）恒安全。
#pragma once

namespace gui {

// prev/cur 单位 °。相邻两拍角度差超过半圈即判定整圈回绕，补偿 ±65536° 使累计角连续。
inline double unwrap_angle(double prev, double cur) noexcept {
    double delta = cur - prev;
    if (delta > 32768.0) {
        delta -= 65536.0;
    } else if (delta < -32768.0) {
        delta += 65536.0;
    }
    return prev + delta;
}

}  // namespace gui
