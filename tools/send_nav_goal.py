#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Nav Goal Sender (交互式导航目标点发送器)
- 终端交互式输入目标点（支持预设编号或自定义 X Y [Yaw] 坐标）
- 回车后立即发起 Nav2 自主导航动作并实时跟踪距离与状态
- 支持随时取消导航，便于快速复现与可视化贴墙/撞墙现象
"""

import sys
import math
import time
import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# 2027 赛场常用关键点预设 (x, y, yaw_deg, description)
PRESET_GOALS = {
    "1": (0.0, 4.50, 0.0, "五色石北侧开阔巡检点 (核心任务点)"),
    "2": (-4.50, 0.00, 90.0, "西侧走廊中段 (红方主跑道)"),
    "3": (-4.50, 4.50, 0.0, "西北转角区 (直道转弯测试)"),
    "4": (4.50, 4.50, -90.0, "东北转角区 (直道转弯测试)"),
    "5": (4.50, 0.00, -90.0, "东侧走廊中段 (蓝方主跑道)"),
    "6": (0.0, -4.50, 0.0, "南侧走廊横向中点 (贯通测试点)"),
    "7": (-4.75, -5.00, 90.0, "红方出发区 (西南角起点)"),
    "8": (4.75, -5.00, 90.0, "蓝方出发区 (东南角起点)"),
    "9": (-5.00, 0.00, 90.0, "西侧临界贴墙区 (贴墙压力测试点)"),
    "10": (5.00, 0.00, -90.0, "东侧临界贴墙区 (贴墙压力测试点)"),
}


def quaternion_from_yaw(yaw_rad):
    """航向角转四元数 (绕 Z 轴旋转)"""
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class InteractiveGoalSender(Node):
    def __init__(self, namespace="AT_R2", use_sim_time=True):
        super().__init__("interactive_goal_sender")

        self.set_parameters([
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, use_sim_time)
        ])

        self.ns = namespace.strip("/")
        self.prefix = f"/{self.ns}" if self.ns else ""

        # Nav2 NavigateToPose Action 客户端
        action_name = f"{self.prefix}/navigate_to_pose"
        self._action_client = ActionClient(self, NavigateToPose, action_name)

        # 同时发布到 /goal_pose 话题供 RViz 显示目标箭头
        topic_name = f"{self.prefix}/goal_pose"
        self._goal_pub = self.create_publisher(PoseStamped, topic_name, 10)

        self._current_goal_handle = None
        self._is_navigating = False
        self._distance_remaining = None
        self._nav_start_time = None
        self._lock = threading.Lock()

    def wait_for_server(self, timeout_sec=5.0):
        action_name = f"{self.prefix}/navigate_to_pose"
        print(f"[INFO] 正在连接 Nav2 动作服务器: {action_name} ...")
        connected = self._action_client.wait_for_server(timeout_sec=timeout_sec)
        if connected:
            print("\033[1;32m[SUCCESS] 成功连接到 Nav2 导航服务器！\033[0m")
        else:
            print("\033[1;33m[WARNING] 动作服务器暂未就绪，可先输入目标，发送时将继续等待连接。\033[0m")
        return connected

    def send_goal(self, x, y, yaw_deg):
        """向 Nav2 发送目标位姿"""
        yaw_rad = math.radians(yaw_deg)
        qx, qy, qz, qw = quaternion_from_yaw(yaw_rad)

        # 构造 PoseStamped
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # 同步发布到 /goal_pose 话题供 RViz 可视化
        self._goal_pub.publish(goal_msg.pose)

        print(f"\n\033[1;34m>>> 正在下发目标点: X={x:+.3f}m, Y={y:+.3f}m, Yaw={yaw_deg:+.1f}° ({yaw_rad:+.3f} rad)\033[0m")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("\033[1;31m[ERROR] 导航服务器未响应，请检查 Nav2 是否成功运行！\033[0m")
            return False

        with self._lock:
            self._is_navigating = True
            self._distance_remaining = None
            self._nav_start_time = time.time()

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("\033[1;31m[ERROR] 目标点被 Nav2 拒绝 (可能位于致命障碍区或无法规划路径)！\033[0m")
            with self._lock:
                self._is_navigating = False
            return

        with self._lock:
            self._current_goal_handle = goal_handle

        print("\033[1;32m[INFO] Nav2 目标点已被接受，自主导航开始！(可观察监控终端中的速度和位置)\033[0m")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        with self._lock:
            self._distance_remaining = feedback.distance_remaining
        dist_str = f"{feedback.distance_remaining:.2f}m" if feedback.distance_remaining is not None else "未知"
        elapsed = time.time() - self._nav_start_time if self._nav_start_time else 0.0
        sys.stdout.write(f"\r  \033[1;36m[导航中]\033[0m 剩余距离: \033[1;33m{dist_str}\033[0m | 已耗时: \033[1;32m{elapsed:.1f}s\033[0m (输入 c 取消)   ")
        sys.stdout.flush()

    def _get_result_callback(self, future):
        result = future.result()
        status = result.status
        sys.stdout.write("\r" + " " * 80 + "\r")
        with self._lock:
            self._is_navigating = False
            self._current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("\n\033[1;42;37m  [成功] 机器人已顺利到达目标点！  \033[0m\n")
        elif status == GoalStatus.STATUS_CANCELED:
            print("\n\033[1;43;30m  [提示] 导航任务已被用户取消。  \033[0m\n")
        elif status == GoalStatus.STATUS_ABORTED:
            print("\n\033[1;41;37m  [中止] 导航任务中止！(可能遇到碰撞、脱轨或卡死)  \033[0m\n")
        else:
            print(f"\n\033[1;31m[INFO] 导航结束，状态码: {status}\033[0m\n")

    def cancel_goal(self):
        with self._lock:
            handle = self._current_goal_handle
        if handle is not None:
            print("\n[INFO] 正在取消当前导航动作...")
            handle.cancel_goal_async()
            return True
        else:
            print("\n[INFO] 当前没有正在执行的导航任务。")
            return False

    def is_navigating(self):
        with self._lock:
            return self._is_navigating


def print_menu():
    print("\n" + "=" * 72)
    print("  \033[1;36m2027 赛场快速导航预设目标点列表:\033[0m")
    for key, (x, y, yaw, desc) in sorted(PRESET_GOALS.items(), key=lambda item: int(item[0])):
        print(f"    [\033[1;33m{key:>2}\033[0m] {desc:<24} -> (X: {x:+5.2f}, Y: {y:+5.2f}, Yaw: {yaw:+5.1f}°)")
    print("-" * 72)
    print("  \033[1;32m输入方式:\033[0m")
    print("    1. 输入预设序号直接回车 (例如: \033[1;33m1\033[0m 或 \033[1;33m3\033[0m)")
    print("    2. 输入自定义坐标 (例如: \033[1;33m0.0 3.5\033[0m 或 \033[1;33m-2.5 1.0 90\033[0m)")
    print("    3. 输入 \033[1;31mc\033[0m 或 \033[1;31mcancel\033[0m 取消当前导航")
    print("    4. 输入 \033[1;31mq\033[0m 或 \033[1;31mquit\033[0m 退出程序")
    print("=" * 72)


def parse_input(user_input):
    user_input = user_input.strip()
    if not user_input:
        return None

    # 预设检查
    if user_input in PRESET_GOALS:
        x, y, yaw, _ = PRESET_GOALS[user_input]
        return x, y, yaw

    # 自定义坐标解析
    parts = user_input.split()
    if len(parts) >= 2:
        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else 0.0
            return x, y, yaw
        except ValueError:
            return None

    return None


def main():
    parser = argparse.ArgumentParser(description="交互式发送导航目标点")
    parser.add_argument("--ns", "--namespace", default="AT_R2", help="机器人命名空间 (默认: AT_R2)")
    parser.add_argument("--no-sim-time", action="store_true", help="禁用仿真时间")
    args = parser.parse_args()

    rclpy.init()
    node = InteractiveGoalSender(namespace=args.ns, use_sim_time=not args.no_sim_time)

    def spin_worker():
        try:
            rclpy.spin(node)
        except Exception:
            pass

    # 启动后台 ROS 2 事件循环线程
    spin_thread = threading.Thread(target=spin_worker, daemon=True)
    spin_thread.start()

    node.wait_for_server(timeout_sec=3.0)
    print_menu()

    try:
        while rclpy.ok():
            try:
                raw = input("\n\033[1;37m请输入目标点编号或坐标 [x y (yaw)]: \033[0m")
            except EOFError:
                break

            cmd = raw.strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("c", "cancel"):
                node.cancel_goal()
                continue
            elif cmd in ("m", "menu", "help", "h"):
                print_menu()
                continue

            target = parse_input(raw)
            if target is None:
                print("\033[1;31m[错误] 无法识别的输入，请输入序号 (1-10) 或坐标 (如: 0 3.5 90)，输入 m 查看菜单。\033[0m")
                continue

            x, y, yaw = target
            node.send_goal(x, y, yaw)

            # 等待导航完成或用户输入取消
            while node.is_navigating():
                time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[INFO] 接收到退出信号，正在取消导航...")
        node.cancel_goal()
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
        print("[INFO] 目标发送器已退出。")


if __name__ == "__main__":
    main()
