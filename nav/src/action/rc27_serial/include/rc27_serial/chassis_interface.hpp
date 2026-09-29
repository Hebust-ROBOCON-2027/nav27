// 底盘抽象接口（舵轮版）
//
// 上层（ROS 节点 / 决策层）只依赖这个接口，不关心底层是串口、CAN 还是仿真。
// 换通信方式时只要换一个实现类，上层代码一行不动。
#pragma once

#include <chrono>
#include <cstdint>

#include "rc27_serial/protocol.hpp"

namespace rc27 {

/// 舵轮状态：每个轮的「当前转向角 + 当前轮速」
struct SteerState {
    float angle[STEER_WHEEL_COUNT]{};  // rad，0 = 与车头同向，逆时针为正
    float speed[STEER_WHEEL_COUNT]{};  // m/s
    std::chrono::steady_clock::time_point stamp{};
    bool valid{false};
};

class IChassis {
public:
    virtual ~IChassis() = default;

    /// 下发车体速度：vx 前(m/s)、vy 左(m/s)、wz 逆时针角速度(rad/s)
    /// 舵轮逆运动学（车体速度 -> 各轮转向角+转速）由实现类/固件完成
    virtual bool setVelocity(float vx, float vy, float wz) = 0;

    /// 急停
    virtual bool emergencyStop() = 0;

    /// 取最近一次舵轮状态反馈
    virtual SteerState getSteerState() const = 0;

    /// 链路是否可用
    virtual bool isConnected() const = 0;
};

}  // namespace rc27
