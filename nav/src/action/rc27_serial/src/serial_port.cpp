#include "rc27_serial/serial_port.hpp"

#include <asm-generic/termbits.h>  // struct termios2 / BOTHER / CBAUD（不引入 glibc termios.h 避免类型冲突）
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <thread>
#include <vector>

#ifndef TCGETS2
#define TCGETS2 0x802C542A
#endif
#ifndef TCSETS2
#define TCSETS2 0x402C542B
#endif
#ifndef TCFLSH
#define TCFLSH 0x540B
#endif

namespace rc27 {

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& port, int baudrate) {
    port_ = port;
    baudrate_ = baudrate;

    if (!openPort()) {
        return false;
    }

    parser_.reset();
    connected_.store(true);
    running_.store(true);
    recv_thread_ = std::thread(&SerialPort::recvThreadFunc, this);
    return true;
}

void SerialPort::close() {
    running_.store(false);
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    connected_.store(false);
}

bool SerialPort::isOpen() const { return connected_.load() && fd_ >= 0; }

std::string SerialPort::lastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

bool SerialPort::openPort() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        setLastError("open " + port_ + " failed: " + std::strerror(errno));
        return false;
    }
    if (!configurePort(baudrate_)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    ioctl(fd_, TCFLSH, TCIFLUSH);
    return true;
}

bool SerialPort::configurePort(int baudrate) {
    struct termios2 tio {};
    if (ioctl(fd_, TCGETS2, &tio) < 0) {
        setLastError("TCGETS2 failed: " + std::string(std::strerror(errno)));
        return false;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    // 8N1：8 数据位、无校验、1 停止位
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;

    tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | ISTRIP | BRKINT);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);
    tio.c_oflag &= ~OPOST;

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (ioctl(fd_, TCSETS2, &tio) < 0) {
        setLastError("TCSETS2 (baud=" + std::to_string(baudrate) + ") failed: " +
                     std::strerror(errno));
        return false;
    }
    return true;
}

bool SerialPort::writeAll(const uint8_t* data, size_t size) {
    size_t written = 0;
    while (written < size) {
        const ssize_t n = ::write(fd_, data + written, size - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        setLastError("write failed: " + std::string(std::strerror(errno)));
        return false;
    }
    return true;
}

bool SerialPort::sendFrame(uint8_t cmd, const uint8_t* data, size_t len) {
    if (len > MAX_DATA_SIZE) {
        setLastError("data too large: " + std::to_string(len));
        return false;
    }

    // | 帧头2 | 命令字1 | 长度1 | 数据N | CRC16 2 |
    const size_t frame_size = len + FRAME_OVERHEAD;
    std::vector<uint8_t> frame(frame_size, 0);
    frame[0] = FRAME_HEADER_0;
    frame[1] = FRAME_HEADER_1;
    frame[2] = cmd;
    frame[3] = static_cast<uint8_t>(len);
    if (len > 0 && data != nullptr) {
        std::memcpy(frame.data() + 4, data, len);
    }

    const uint16_t crc =
        crc16Modbus(frame.data() + CRC_START_OFFSET, frame_size - 2 - CRC_START_OFFSET);
    frame[frame_size - 2] = static_cast<uint8_t>(crc & 0xFF);         // CRC 低字节在前
    frame[frame_size - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (!connected_.load() || fd_ < 0) {
            send_fail_count_.fetch_add(1, std::memory_order_relaxed);
            setLastError("send skipped: link down");
            return false;
        }
        if (!writeAll(frame.data(), frame.size())) {
            send_fail_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
    }

    tx_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void SerialPort::setFrameCallback(FrameCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = std::move(callback);
}

RingParser::Stats SerialPort::consumeParseStats() { return parser_.consumeStats(); }

void SerialPort::recvThreadFunc() {
    uint8_t buffer[2048];

    while (running_.load()) {
        if (!connected_.load()) {
            tryReconnect();
            continue;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);
        struct timeval timeout {};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 100ms

        const int ret = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            setLastError("select error: " + std::string(std::strerror(errno)));
            ::close(fd_);
            fd_ = -1;
            connected_.store(false);
            continue;
        }
        if (ret == 0) {
            continue;
        }

        const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
        if (n > 0) {
            parser_.push(buffer, static_cast<size_t>(n));
            parser_.parse([this](uint8_t cmd, const uint8_t* data, size_t len) {
                rx_count_.fetch_add(1, std::memory_order_relaxed);
                FrameCallback callback;
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    callback = frame_callback_;
                }
                if (callback) {
                    callback(cmd, data, len);
                }
            });
        } else if (n == 0) {
            setLastError("serial EOF, device disconnected");
            ::close(fd_);
            fd_ = -1;
            connected_.store(false);
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            setLastError("read error: " + std::string(std::strerror(errno)));
            ::close(fd_);
            fd_ = -1;
            connected_.store(false);
        }
    }
}

void SerialPort::tryReconnect() {
    std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_INTERVAL_MS));
    if (!running_.load()) {
        return;
    }
    if (openPort()) {
        parser_.reset();
        connected_.store(true);
        reconnect_count_.fetch_add(1, std::memory_order_relaxed);
        setLastError("");
    }
}

void SerialPort::setLastError(const std::string& message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
}

}  // namespace rc27
