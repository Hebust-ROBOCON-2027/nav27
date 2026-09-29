// R2 协议 v2.2 串口 I/O 层
//
// 只管字节流：打开/配置串口、组帧、写帧、后台读线程流式解析、断线重连。
// 不认识 cmd_vel，不做限速——那是 IChassis 实现层和 ROS 节点的事。
//
// 波特率用 termios2 + BOTHER 设置，115200 及任意值都支持。
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "rc27_serial/protocol.hpp"
#include "rc27_serial/ring_parser.hpp"

namespace rc27 {

class SerialPort {
public:
    using FrameCallback = std::function<void(uint8_t cmd, const uint8_t* data, size_t len)>;

    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& port, int baudrate = UART_BAUDRATE);
    void close();
    bool isOpen() const;
    std::string lastError() const;

    /// 组帧并发送：帧头 + 命令字 + 长度 + 数据 + CRC16
    bool sendFrame(uint8_t cmd, const uint8_t* data = nullptr, size_t len = 0);

    /// 注意：回调运行在接收线程内，禁止在回调里调用 sendFrame / open / close
    void setFrameCallback(FrameCallback callback);

    uint64_t txCount() const { return tx_count_.load(std::memory_order_relaxed); }
    uint64_t rxCount() const { return rx_count_.load(std::memory_order_relaxed); }
    uint64_t sendFailCount() const { return send_fail_count_.load(std::memory_order_relaxed); }
    uint32_t reconnectCount() const { return reconnect_count_.load(std::memory_order_relaxed); }

    RingParser::Stats consumeParseStats();

private:
    void recvThreadFunc();
    bool openPort();
    bool configurePort(int baudrate);
    bool writeAll(const uint8_t* data, size_t size);
    void setLastError(const std::string& message);
    void tryReconnect();

    int fd_{-1};
    std::string port_;
    int baudrate_{UART_BAUDRATE};

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread recv_thread_;

    std::mutex write_mutex_;
    mutable std::mutex error_mutex_;
    std::string last_error_;

    mutable std::mutex callback_mutex_;
    FrameCallback frame_callback_;

    RingParser parser_;

    std::atomic<uint64_t> tx_count_{0};
    std::atomic<uint64_t> rx_count_{0};
    std::atomic<uint64_t> send_fail_count_{0};
    std::atomic<uint32_t> reconnect_count_{0};
};

}  // namespace rc27
