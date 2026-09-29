// RC2027 底盘桥接节点（R2 协议 v2.2 / 舵轮底盘）
//
// 依赖倒置：节点只持有 IChassis 接口，不关心底层是串口、CAN 还是仿真。
// 换通信方式时只换一个实现类，本文件不用动。
//
// 安全策略：
//   - 单轴限幅 + 合速度按模长等比缩放（保持方向不畸变）
//   - cmd_vel 超时后改发零速，重复 N 次后停发；可选改成发急停帧
//   - 串口断开自动重连
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <utility>

#include "rc27_serial/chassis_interface.hpp"
#include "rc27_serial/protocol.hpp"
#include "rc27_serial/serial_chassis.hpp"

class ChassisBridgeNode : public rclcpp::Node {
public:
    ChassisBridgeNode() : Node("chassis_bridge_node") {
        // ---------------- 参数 ----------------
        const std::string port = this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        const int baudrate = this->declare_parameter<int>("baudrate", 115200);
        cmd_vel_topic_ =
            this->declare_parameter<std::string>("cmd_vel_topic", "/AT_R2/cmd_vel_nav2_result");
        send_rate_hz_ = this->declare_parameter<int>("send_rate_hz", 50);
        cmd_vel_timeout_ms_ = this->declare_parameter<int>("cmd_vel_timeout_ms", 200);
        stop_repeat_n_ = this->declare_parameter<int>("stop_repeat_n", 10);
        v_max_mps_ = this->declare_parameter<double>("v_max_mps", 3.0);
        w_max_radps_ = this->declare_parameter<double>("w_max_radps", 5.0);
        swap_xy_ = this->declare_parameter<bool>("swap_xy", false);
        invert_vx_ = this->declare_parameter<bool>("invert_vx", false);
        invert_vy_ = this->declare_parameter<bool>("invert_vy", false);
        invert_wz_ = this->declare_parameter<bool>("invert_wz", false);
        estop_on_timeout_ = this->declare_parameter<bool>("estop_on_timeout", false);

        send_rate_hz_ = std::max(1, send_rate_hz_);
        cmd_vel_timeout_ms_ = std::max(0, cmd_vel_timeout_ms_);

        // ---------------- 底盘（接口背后是串口实现） ----------------
        chassis_ = std::make_unique<rc27::SerialChassis>(port, baudrate);
        chassis_if_ = chassis_.get();  // 业务调用一律走接口

        if (chassis_if_->isConnected()) {
            RCLCPP_INFO(get_logger(), "串口已打开: %s @ %d", port.c_str(), baudrate);
        } else {
            RCLCPP_WARN(get_logger(), "串口打开失败 (%s)，将持续重连: %s", port.c_str(),
                        chassis_->lastError().c_str());
        }

        // ---------------- ROS 接口 ----------------
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->onCmdVel(msg); });

        steer_state_pub_ =
            this->create_publisher<std_msgs::msg::Float32MultiArray>("/AT_R2/steer_state", 10);

        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / send_rate_hz_),
                                              [this]() { this->onSendTick(); });
        report_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                [this]() { this->onReportTick(); });
        stats_timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                               [this]() { this->onStatsTick(); });

        RCLCPP_INFO(get_logger(),
                    "底盘桥接已启动: 订阅 %s, 发送 %d Hz, 超时 %d ms | 舵轮底盘 vx/vy/wz, "
                    "限速 v=%.2f m/s w=%.2f rad/s",
                    cmd_vel_topic_.c_str(), send_rate_hz_, cmd_vel_timeout_ms_, v_max_mps_,
                    w_max_radps_);
    }

private:
    // ---------------- cmd_vel -> 车体速度指令 ----------------
    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double wz = msg->angular.z;

        if (!std::isfinite(vx)) vx = 0.0;
        if (!std::isfinite(vy)) vy = 0.0;
        if (!std::isfinite(wz)) wz = 0.0;

        // 合速度限幅：按模长等比缩放，保持方向
        const double linear_limit = std::fabs(v_max_mps_);
        const double mag = std::hypot(vx, vy);
        if (linear_limit <= 0.0) {
            vx = 0.0;
            vy = 0.0;
        } else if (mag > linear_limit && mag > 0.0) {
            const double scale = linear_limit / mag;
            vx *= scale;
            vy *= scale;
        }
        const double angular_limit = std::fabs(w_max_radps_);
        wz = (angular_limit <= 0.0) ? 0.0 : std::clamp(wz, -angular_limit, angular_limit);

        // 装机方向不对改参数即可，不用改代码重编译
        if (swap_xy_) std::swap(vx, vy);
        if (invert_vx_) vx = -vx;
        if (invert_vy_) vy = -vy;
        if (invert_wz_) wz = -wz;

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        target_vx_ = static_cast<float>(vx);
        target_vy_ = static_cast<float>(vy);
        target_wz_ = static_cast<float>(wz);
        last_cmd_time_ = std::chrono::steady_clock::now();
        command_received_ = true;
        timeout_zero_remaining_ = 0;
        timeout_zero_done_ = false;
        estop_sent_ = false;
    }

    // ---------------- 周期下发 ----------------
    void onSendTick() {
        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
        bool send = false;
        bool estop = false;

        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            if (!command_received_) {
                return;  // 从没收到过 cmd_vel，不打扰下位机
            }

            const auto now = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::milliseconds(cmd_vel_timeout_ms_);
            if (cmd_vel_timeout_ms_ == 0 || now - last_cmd_time_ <= timeout) {
                timeout_zero_remaining_ = 0;
                vx = target_vx_;
                vy = target_vy_;
                wz = target_wz_;
                send = true;
            } else if (estop_on_timeout_) {
                if (!estop_sent_) {
                    estop_sent_ = true;
                    estop = true;
                    send = true;
                }
            } else if (!timeout_zero_done_) {
                if (timeout_zero_remaining_ == 0) {
                    timeout_zero_remaining_ = stop_repeat_n_;
                }
                if (timeout_zero_remaining_ > 0) {
                    --timeout_zero_remaining_;
                    if (timeout_zero_remaining_ == 0) {
                        timeout_zero_done_ = true;
                    }
                    send = true;  // 零速
                }
            }
        }

        if (!send) {
            return;
        }

        const bool ok = estop ? chassis_if_->emergencyStop() : chassis_if_->setVelocity(vx, vy, wz);
        if (!ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "发送失败: %s",
                                 chassis_->lastError().c_str());
        }
    }

    // ---------------- 舵轮状态反馈 ----------------
    void onReportTick() {
        const rc27::SteerState st = chassis_if_->getSteerState();
        if (!st.valid) {
            return;
        }
        std_msgs::msg::Float32MultiArray msg;
        // 前 4 个是转向角(rad)，后 4 个是轮速(m/s)
        for (size_t i = 0; i < rc27::STEER_WHEEL_COUNT; ++i) {
            msg.data.push_back(st.angle[i]);
        }
        for (size_t i = 0; i < rc27::STEER_WHEEL_COUNT; ++i) {
            msg.data.push_back(st.speed[i]);
        }
        steer_state_pub_->publish(msg);
    }

    // ---------------- 链路健康度 ----------------
    void onStatsTick() {
        const auto stats = chassis_->consumeParseStats();
        RCLCPP_INFO(get_logger(),
                    "链路 %s | 发 %lu 收 %lu | 发失败 %lu 重连 %u | 废帧 头%u 长%u CRC%u",
                    chassis_if_->isConnected() ? "已连接" : "断开", chassis_->txCount(),
                    chassis_->rxCount(), chassis_->sendFailCount(), chassis_->reconnectCount(),
                    stats.head_drop, stats.len_invalid, stats.crc_bad);
        if (stats.crc_bad > 0 || stats.len_invalid > 0) {
            RCLCPP_WARN(get_logger(),
                        "检测到废帧：确认波特率 115200、CRC-16/Modbus，以及 CRC 是否包含帧头"
                        "（protocol.hpp 里 CRC_START_OFFSET）两侧一致");
        }
    }

    // ---------------- 成员 ----------------
    // 具体实现：额外提供 lastError / 帧统计等诊断能力
    std::unique_ptr<rc27::SerialChassis> chassis_;
    // 业务逻辑只依赖接口：换通信方式时只改上面那行的实现类
    rc27::IChassis* chassis_if_{nullptr};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr steer_state_pub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr report_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::string cmd_vel_topic_;
    int send_rate_hz_{50};
    int cmd_vel_timeout_ms_{200};
    int stop_repeat_n_{10};
    double v_max_mps_{3.0};
    double w_max_radps_{5.0};
    bool swap_xy_{false};
    bool invert_vx_{false};
    bool invert_vy_{false};
    bool invert_wz_{false};
    bool estop_on_timeout_{false};

    std::mutex cmd_mutex_;
    float target_vx_{0.0f};
    float target_vy_{0.0f};
    float target_wz_{0.0f};
    std::chrono::steady_clock::time_point last_cmd_time_{};
    bool command_received_{false};
    int timeout_zero_remaining_{0};
    bool timeout_zero_done_{false};
    bool estop_sent_{false};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
