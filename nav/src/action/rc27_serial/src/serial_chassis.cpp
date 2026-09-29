#include "rc27_serial/serial_chassis.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rc27 {

SerialChassis::SerialChassis(std::string port, int baudrate) {
    port_.setFrameCallback(
        [this](uint8_t cmd, const uint8_t* data, size_t len) { this->onFrame(cmd, data, len); });
    port_.open(std::move(port), baudrate);
}

SerialChassis::~SerialChassis() { port_.close(); }

bool SerialChassis::setVelocity(float vx, float vy, float wz) {
    // 单轴限幅
    if (!std::isfinite(vx)) vx = 0.0f;
    if (!std::isfinite(vy)) vy = 0.0f;
    if (!std::isfinite(wz)) wz = 0.0f;
    vx = std::clamp(vx, -LINEAR_AXIS_LIMIT, LINEAR_AXIS_LIMIT);
    vy = std::clamp(vy, -LINEAR_AXIS_LIMIT, LINEAR_AXIS_LIMIT);
    wz = std::clamp(wz, -ANGULAR_LIMIT, ANGULAR_LIMIT);

    // 合速度限幅：按模长等比缩放，保持方向不畸变
    const float mag = std::hypot(vx, vy);
    if (mag > LINEAR_MAG_LIMIT && mag > 0.0f) {
        const float scale = LINEAR_MAG_LIMIT / mag;
        vx *= scale;
        vy *= scale;
    }

    SpeedCmdData cmd;
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.wz = wz;

    uint8_t data[SPEED_CMD_DATA_SIZE];
    std::memcpy(data + 0, &cmd.vx, sizeof(float));
    std::memcpy(data + 4, &cmd.vy, sizeof(float));
    std::memcpy(data + 8, &cmd.wz, sizeof(float));

    // 舵轮逆运动学（车体速度 -> 各轮转向角+转速）由电控固件完成
    return port_.sendFrame(static_cast<uint8_t>(CommandID::SPEED_CMD), data, sizeof(data));
}

bool SerialChassis::emergencyStop() {
    // 0x02 急停：无数据区
    return port_.sendFrame(static_cast<uint8_t>(CommandID::ESTOP));
}

SteerState SerialChassis::getSteerState() const {
    std::lock_guard<std::mutex> lock(steer_mutex_);
    return steer_state_;
}

bool SerialChassis::isConnected() const { return port_.isOpen(); }

void SerialChassis::onFrame(uint8_t cmd, const uint8_t* data, size_t len) {
    // 上行只有 0x10 舵轮状态；其余（含下行命令回环）一律忽略
    if (cmd != static_cast<uint8_t>(CommandID::STEER_STATE)) {
        return;
    }
    if (len != STEER_WHEEL_DATA_SIZE) {
        return;
    }

    // 前 16B 是 4 个转向角，后 16B 是 4 个轮速
    SteerWheelData sw;
    std::memcpy(sw.angle, data + 0, sizeof(sw.angle));
    std::memcpy(sw.speed, data + sizeof(sw.angle), sizeof(sw.speed));

    std::lock_guard<std::mutex> lock(steer_mutex_);
    for (size_t i = 0; i < STEER_WHEEL_COUNT; ++i) {
        steer_state_.angle[i] = sw.angle[i];
        steer_state_.speed[i] = sw.speed[i];
    }
    steer_state_.stamp = std::chrono::steady_clock::now();
    steer_state_.valid = true;
}

}  // namespace rc27
