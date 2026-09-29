#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot State Monitor (实时位姿与速度监控器)
- 实时获取并打印机器人位于 map 坐标系下的 X, Y, Z 与航向角 Yaw
- 实时监控机器人 X, Y 方向速度（Nav2 指令速度与仿真 odom 反馈速度）
- 专用于观察导航贴墙、撞墙与速度突变现象
"""

import sys
import math
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def euler_from_quaternion(x, y, z, w):
    """四元数转航向角 (Roll, Pitch, Yaw)"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class RobotStateMonitor(Node):
    def __init__(self, namespace="AT_R2", use_sim_time=True):
        super().__init__("robot_state_monitor")

        self.set_parameters([
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, use_sim_time)
        ])

        self.ns = namespace.strip("/")
        self.prefix = f"/{self.ns}" if self.ns else ""

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.prefix:
            static_qos = QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(TFMessage, f"{self.prefix}/tf", self.tf_callback, 100)
            self.create_subscription(TFMessage, f"{self.prefix}/tf_static", self.tf_static_callback, static_qos)

        # 状态记录
        self.pose_map = None
        self.yaw_deg = 0.0
        self.yaw_rad = 0.0
        self.target_frame = "base_footprint"
        self.available_frames = ["base_footprint", "chassis", "base_link"]

        # 速度记录: Nav2 下发指令
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.cmd_time = 0.0

        # 速度记录: Odom 反馈 (仿真实际底盘速度)
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_time = 0.0

        # 时钟监控
        self.sim_time_sec = 0.0
        self.clock_received = False

        # 订阅话题
        cmd_topic = f"{self.prefix}/cmd_vel_nav2_result"
        odom_topic = f"{self.prefix}/odom"

        self.create_subscription(Twist, cmd_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Twist, f"{self.prefix}/cmd_vel", self.cmd_vel_fallback_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Clock, "/clock", self.clock_callback, 10)

        # 定时打印定时器 (10 Hz)
        self.timer = self.create_timer(0.1, self.update_and_render)

    def clock_callback(self, msg: Clock):
        self.sim_time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_received = True

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self.cmd_time = time.time()

    def cmd_vel_fallback_callback(self, msg: Twist):
        if time.time() - self.cmd_time > 0.5:
            self.cmd_vx = msg.linear.x
            self.cmd_vy = msg.linear.y
            self.cmd_wz = msg.angular.z

    def odom_callback(self, msg: Odometry):
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vy = msg.twist.twist.linear.y
        self.odom_wz = msg.twist.twist.angular.z
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_time = time.time()

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, "default_authority")

    def tf_static_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, "default_authority")

    def query_tf(self):
        """尝试从 map 查询机器人的全局坐标"""
        for frame in self.available_frames:
            try:
                # 优先查询最新 transform
                trans = self.tf_buffer.lookup_transform(
                    "map",
                    frame,
                    rclpy.time.Time()
                )
                self.target_frame = frame
                t = trans.transform.translation
                r = trans.transform.rotation
                _, _, yaw = euler_from_quaternion(r.x, r.y, r.z, r.w)
                self.pose_map = (t.x, t.y, t.z)
                self.yaw_rad = yaw
                self.yaw_deg = math.degrees(yaw)
                return True
            except Exception:
                continue
        return False

    def update_and_render(self):
        has_tf = self.query_tf()

        # 计算合成线速度
        cmd_speed = math.hypot(self.cmd_vx, self.cmd_vy)
        odom_speed = math.hypot(self.odom_vx, self.odom_vy)

        now = time.time()
        cmd_active = (now - self.cmd_time < 0.5) if self.cmd_time > 0 else False
        odom_active = (now - self.odom_time < 0.5) if self.odom_time > 0 else False

        # 终端 ANSI 清屏并复位光标
        sys.stdout.write("\033[H\033[2J")

        lines = []
        lines.append("=" * 72)
        lines.append(f"   \033[1;36m2027 赛场导航机器人实时状态监控 (Robot State Monitor)\033[0m")
        lines.append("=" * 72)

        # 1. 系统与时钟状态
        if self.clock_received:
            clock_str = f"\033[1;32m{self.sim_time_sec:8.2f}s (仿真时钟同步正常)\033[0m"
        else:
            clock_str = "\033[1;33m未收到 /clock (请确认仿真是否已启动)\033[0m"
        lines.append(f"  命名空间: \033[1;33m{self.prefix or '/'}\033[0m   |   仿真时钟: {clock_str}")
        lines.append("-" * 72)

        # 2. 全局位置 (map 坐标系)
        lines.append(f"  \033[1;35m[1. 全局位姿 - Map 坐标系 (参考点: {self.target_frame})]\033[0m")
        if has_tf and self.pose_map is not None:
            x, y, z = self.pose_map
            # 对贴墙边界进行预警判断 (场地尺寸 11.1m x 11.1m, 外围边缘在 ±5.55m)
            wall_dist_x = 5.55 - abs(x)
            wall_dist_y = 5.55 - abs(y)
            min_wall_dist = min(wall_dist_x, wall_dist_y)

            if min_wall_dist < 0.6:
                wall_warn = f"\033[1;41;37m [警报: 距外墙仅 {min_wall_dist:.2f}m! 极高贴墙/撞墙风险] \033[0m"
            elif min_wall_dist < 0.9:
                wall_warn = f"\033[1;33m [注意: 距外墙 {min_wall_dist:.2f}m 临近贴墙区] \033[0m"
            else:
                wall_warn = f"\033[1;32m [安全: 距外墙 {min_wall_dist:.2f}m] \033[0m"

            lines.append(f"    X 坐标:  \033[1;32m{x:+7.3f}\033[0m m      (红方出发: -4.75, 蓝方出发: +4.75)")
            lines.append(f"    Y 坐标:  \033[1;32m{y:+7.3f}\033[0m m      (红/蓝出发: -5.00, 五色石: +4.25)")
            lines.append(f"    Z 坐标:  \033[1;32m{z:+7.3f}\033[0m m      (地面: 0.15m, 一层平台: 0.75m)")
            lines.append(f"    航向角:  \033[1;32m{self.yaw_deg:+7.2f}°\033[0m ({self.yaw_rad:+.4f} rad) {wall_warn}")
        else:
            lines.append("    \033[1;31m[等待 map -> base TF 发布中... 请确认 small_gicp 重定位或 LIO 正常启动]\033[0m")
        lines.append("-" * 72)

        # 3. 速度监控: Nav2 指令下发速度
        lines.append(f"  \033[1;34m[2. 下发控制指令 - Nav2 MPPI 控制器 (/cmd_vel_nav2_result)]\033[0m")
        if cmd_active:
            lines.append(f"    Vx (车头前向速度): \033[1;33m{self.cmd_vx:+6.3f}\033[0m m/s")
            lines.append(f"    Vy (车身横向速度): \033[1;33m{self.cmd_vy:+6.3f}\033[0m m/s  (全向麦轮侧移)")
            lines.append(f"    合线速度模 |V|:    \033[1;33m{cmd_speed:6.3f}\033[0m m/s")
            lines.append(f"    Wz (自转角速度):   \033[1;33m{self.cmd_wz:+6.3f}\033[0m rad/s ({math.degrees(self.cmd_wz):+6.1f}°/s)")
        else:
            lines.append("    \033[1;30m当前无 Nav2 活跃指令下发 (静止/等待目标中)\033[0m")
        lines.append("-" * 72)

        # 4. 速度监控: 仿真实际反馈速度 (Odom)
        lines.append(f"  \033[1;36m[3. 底盘物理反馈 - Gazebo 力控插件反馈 (/odom)]\033[0m")
        if odom_active:
            lines.append(f"    反馈实际 Vx:       \033[1;32m{self.odom_vx:+6.3f}\033[0m m/s")
            lines.append(f"    反馈实际 Vy:       \033[1;32m{self.odom_vy:+6.3f}\033[0m m/s")
            lines.append(f"    实际线速度模:      \033[1;32m{odom_speed:6.3f}\033[0m m/s")
            lines.append(f"    反馈实际 Wz:       \033[1;32m{self.odom_wz:+6.3f}\033[0m rad/s ({math.degrees(self.odom_wz):+6.1f}°/s)")
        else:
            lines.append("    \033[1;30m未收到 /odom 速度反馈 (请确认 Gazebo ros_gz_bridge 是否正常启动)\033[0m")

        lines.append("=" * 72)
        lines.append("  \033[1;30m快捷提示: 按 Ctrl+C 退出监控器 | 在目标输入终端发目标点测试\033[0m")

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="实时监控机器人在 map 坐标系下的位姿与 X/Y 方向速度")
    parser.add_argument("--ns", "--namespace", default="AT_R2", help="机器人命名空间 (默认: AT_R2)")
    parser.add_argument("--no-sim-time", action="store_true", help="禁用仿真时间")
    args = parser.parse_args()

    rclpy.init()
    node = RobotStateMonitor(namespace=args.ns, use_sim_time=not args.no_sim_time)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        print("\n[INFO] 机器人状态监控已停止。")


if __name__ == "__main__":
    main()
