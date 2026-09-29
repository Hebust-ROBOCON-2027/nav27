// R2 协议 v2.2 流式解析器（配合 protocol.hpp）
//
// 帧：0xAA 0x55 | CMD | LEN | DATA(N) | CRC16(2)，总长 = LEN + 6
//
// 串口是字节流，一次 read() 可能拿到半帧、一帧半甚至多帧，
// 必须靠环形缓冲攒字节 + 状态机切帧，不能把一次 read 的长度当成一帧。
//
// 切帧规则：帧头 2B -> 读 LEN -> 凑够 LEN+6 -> CRC16 校验。
// 三重判定全过才认为是一帧，误判概率约 1/2^32。
// 任一步失败只丢弃 1 字节重新找帧头，绝不大段丢弃（避免误杀真帧）。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "rc27_serial/protocol.hpp"

namespace rc27 {

class RingParser {
public:
    static constexpr size_t CAPACITY = 4096;

    /// cmd: 命令字; data+len: 数据区
    using DeliverFn = std::function<void(uint8_t cmd, const uint8_t* data, size_t len)>;

    struct Stats {
        uint32_t frames_ok = 0;
        uint32_t len_invalid = 0;
        uint32_t crc_bad = 0;
        uint32_t head_drop = 0;
        uint32_t overflow_drop = 0;
    };

    void push(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            if (size_ >= CAPACITY) {
                drop(1);
                ++stats_.overflow_drop;
            }
            const size_t tail = (head_ + size_) % CAPACITY;
            buf_[tail] = data[i];
            ++size_;
        }
    }

    void parse(const DeliverFn& deliver) {
        while (size() >= MIN_FRAME_SIZE) {
            if (at(0) != FRAME_HEADER_0 || at(1) != FRAME_HEADER_1) {
                drop(1);
                ++stats_.head_drop;
                continue;
            }

            const size_t data_len = at(3);
            if (data_len > MAX_DATA_SIZE) {
                drop(1);
                ++stats_.len_invalid;
                continue;
            }

            const size_t frame_size = data_len + FRAME_OVERHEAD;
            if (frame_size > size()) {
                return;  // 半帧，等新数据
            }

            // CRC 覆盖 [CRC_START_OFFSET, frame_size - 2)
            const uint16_t expected =
                crc16ModbusRange(CRC_START_OFFSET, frame_size - 2 - CRC_START_OFFSET);
            const uint16_t actual =
                static_cast<uint16_t>(at(frame_size - 2)) |
                static_cast<uint16_t>(static_cast<uint16_t>(at(frame_size - 1)) << 8);

            if (expected != actual) {
                drop(1);
                ++stats_.crc_bad;
                continue;
            }

            const uint8_t cmd = at(2);
            std::array<uint8_t, MAX_DATA_SIZE> data{};
            for (size_t i = 0; i < data_len; ++i) {
                data[i] = at(4 + i);
            }

            deliver(cmd, data.data(), data_len);
            drop(frame_size);
            ++stats_.frames_ok;
        }
    }

    void reset() {
        head_ = 0;
        size_ = 0;
        stats_ = {};
    }

    Stats consumeStats() {
        const Stats delta = stats_;
        stats_ = {};
        return delta;
    }

private:
    std::array<uint8_t, CAPACITY> buf_{};
    size_t head_{0};
    size_t size_{0};
    Stats stats_{};

    size_t size() const { return size_; }
    uint8_t at(size_t i) const { return buf_[(head_ + i) % CAPACITY]; }

    /// 对缓冲区中 [start, start+len) 计算 CRC16，避免为算 CRC 额外拷贝
    uint16_t crc16ModbusRange(size_t start, size_t len) const {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i) {
            crc ^= static_cast<uint16_t>(at(start + i));
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    void drop(size_t n) {
        const size_t consumed = (n > size_) ? size_ : n;
        head_ = (head_ + consumed) % CAPACITY;
        size_ -= consumed;
    }
};

}  // namespace rc27
