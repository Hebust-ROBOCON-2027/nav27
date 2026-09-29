// IChassis 的串口实现：把车体速度翻译成 R2 协议 v2.2 的 0x01 帧
#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "rc27_serial/chassis_interface.hpp"
#include "rc27_serial/serial_port.hpp"

namespace rc27 {

class SerialChassis : public IChassis {
public:
    SerialChassis(std::string port, int baudrate = UART_BAUDRATE);
    ~SerialChassis() override;

    bool setVelocity(float vx, float vy, float wz) override;
    bool emergencyStop() override;
    SteerState getSteerState() const override;
    bool isConnected() const override;

    // 供节点做健康度日志
    uint64_t txCount() const { return port_.txCount(); }
    uint64_t rxCount() const { return port_.rxCount(); }
    uint64_t sendFailCount() const { return port_.sendFailCount(); }
    uint32_t reconnectCount() const { return port_.reconnectCount(); }
    RingParser::Stats consumeParseStats() { return port_.consumeParseStats(); }
    std::string lastError() const { return port_.lastError(); }

private:
    void onFrame(uint8_t cmd, const uint8_t* data, size_t len);

    SerialPort port_;

    mutable std::mutex steer_mutex_;
    SteerState steer_state_{};
};

}  // namespace rc27
