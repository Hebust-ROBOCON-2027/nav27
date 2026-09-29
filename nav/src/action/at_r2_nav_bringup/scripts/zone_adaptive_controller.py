#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2026 AT-RC Nav27
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Zone Adaptive Controller (自适应区域导航控制器)
- 实时检测机器人当前位于【平地地面区】还是【窄坡/高台区】(基于 Z 高度与 (X, Y) 区域)
- 动态在两个区域间切换最优的 MPPI 控制器参数与 Costmap 膨胀层参数：
  * 地面模式 (GROUND):
      高速度 (vx_max=2.0, vy_max=1.8), 大膨胀 (0.48m 防高速撞墙), 标准对齐
  * 坡道与高台模式 (RAMP_PLATFORM):
      稳健爬坡速度 (vx_max=0.9), 抑制横移 (vy_max=0.2 防打滑侧翻),
      小膨胀 (0.30m 解开 1 米窄坡死锁), 强路径对齐 (PathAlign=16.0)
- 发布 /<namespace>/current_nav_zone 话题供状态监控与 RViz 可视化
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_msgs.msg import TFMessage


ZONE_GROUND = "GROUND"
ZONE_RAMP_PLATFORM = "RAMP_PLATFORM"


class ZoneAdaptiveController(Node):
    def __init__(self):
        super().__init__("zone_adaptive_controller")

        # 参数声明
        self.declare_parameter("namespace", "AT_R2")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "chassis")
        self.declare_parameter("check_hz", 10.0)
        self.declare_parameter("ground_z_threshold", 0.15)
        self.declare_parameter("ramp_enter_z_threshold", 0.12)
        self.declare_parameter("ground_recover_z_threshold", 0.08)

        ns = self.get_parameter("namespace").value.strip("/")
        self.prefix = f"/{ns}" if ns else ""
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.check_hz = self.get_parameter("check_hz").value
        self.ground_z_thresh = self.get_parameter("ground_z_threshold").value
        self.ramp_enter_z = self.get_parameter("ramp_enter_z_threshold").value
        self.ground_recover_z = self.get_parameter("ground_recover_z_threshold").value

        # 目标服务节点
        self.controller_server_name = f"{self.prefix}/controller_server"
        self.local_costmap_name = f"{self.prefix}/local_costmap/local_costmap"
        self.global_costmap_name = f"{self.prefix}/global_costmap/global_costmap"

        # TF 监听与缓存
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 直接订阅带有命名空间的 /<ns>/tf 和 /<ns>/tf_static，确保即使没有 remapping 也能接收
        self.create_subscription(
            TFMessage, f"{self.prefix}/tf", self._tf_cb, 50
        )
        static_qos = QoSProfile(
            depth=50,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            TFMessage, f"{self.prefix}/tf_static", self._tf_static_cb, static_qos
        )

        # 备选位置源：直接订阅仿真里程计与真值，杜绝 TF 延迟或丢失导致的失控
        self.last_pose = None
        from nav_msgs.msg import Odometry
        self.create_subscription(
            Odometry, f"{self.prefix}/odom", self._odom_cb, 10
        )

        # 状态发布器 (QoS TRANSIENT_LOCAL)
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.zone_pub = self.create_publisher(
            String, f"{self.prefix}/current_nav_zone", status_qos
        )

        # 当前活动状态
        self.current_zone = None
        self.pending_zone = None
        self.pending_count = 0
        self.debounce_threshold = 2  # 连续检测到 2 次才执行切换

        # 参数服务客户端缓存
        self.param_clients = {}

        # 定时器
        self.timer = self.create_timer(1.0 / self.check_hz, self.update_loop)

        # 初始广播
        self._publish_zone(ZONE_GROUND)

        self.get_logger().info(
            f"\033[1;32m[ZoneAdaptiveController] 启动成功！"
            f"目标命名空间: '{self.prefix}', 监控基准: {self.map_frame} -> {self.robot_base_frame}\033[0m"
        )

    def _tf_cb(self, msg: TFMessage):
        for t in msg.transforms:
            try:
                self.tf_buffer.set_transform(t, "zone_ctrl")
            except Exception:
                pass

    def _tf_static_cb(self, msg: TFMessage):
        for t in msg.transforms:
            try:
                self.tf_buffer.set_transform_static(t, "zone_ctrl")
            except Exception:
                pass

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.last_pose = (p.x, p.y, p.z)

    def _get_param_client(self, node_name):
        if node_name not in self.param_clients:
            srv_name = f"{node_name}/set_parameters"
            client = self.create_client(SetParameters, srv_name)
            self.param_clients[node_name] = client
        return self.param_clients[node_name]

    def _set_parameters_async(self, node_name, param_dict):
        """异步向目标节点下发参数修改请求"""
        client = self._get_param_client(node_name)
        if not client.service_is_ready():
            # 尝试连接，不阻塞
            if not client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn(
                    f"[ZoneAdaptive] 服务 {node_name}/set_parameters 暂未就绪，跳过本次设置",
                    throttle_duration_sec=2.0
                )
                return

        req = SetParameters.Request()
        for name, value in param_dict.items():
            param = Parameter()
            param.name = name
            val_msg = ParameterValue()
            if isinstance(value, float):
                val_msg.type = ParameterType.PARAMETER_DOUBLE
                val_msg.double_value = value
            elif isinstance(value, int):
                val_msg.type = ParameterType.PARAMETER_INTEGER
                val_msg.integer_value = value
            elif isinstance(value, bool):
                val_msg.type = ParameterType.PARAMETER_BOOL
                val_msg.bool_value = value
            elif isinstance(value, str):
                val_msg.type = ParameterType.PARAMETER_STRING
                val_msg.string_value = value
            param.value = val_msg
            req.parameters.append(param)

        future = client.call_async(req)

        def _callback(f):
            try:
                res = f.result()
                all_ok = all(r.successful for r in res.results)
                if not all_ok:
                    self.get_logger().warn(f"[ZoneAdaptive] 部分参数设置失败 ({node_name})")
            except Exception as e:
                self.get_logger().error(f"[ZoneAdaptive] 调用 {node_name}/set_parameters 异常: {e}")

        future.add_done_callback(_callback)

    def is_in_ramp_or_platform_box(self, x, y, z):
        """
        判断小车是否位于坡道或高台多边形内
        - 坡道红方 (西侧): X in [-4.3, -2.7], Y in [-1.9, 3.2]
        - 坡道蓝方 (东侧): X in [2.7, 4.3], Y in [-1.9, 3.2]
        - 一阶平台 (L1): X in [-3.2, 3.2], Y in [-3.2, 3.2]
        """
        # 红方坡道及坡顶通道
        if (-4.3 <= x <= -2.7) and (-1.9 <= y <= 3.2):
            return True
        # 蓝方坡道及坡顶通道
        if (2.7 <= x <= 4.3) and (-1.9 <= y <= 3.2):
            return True
        # 一阶平台主体 (且高度脱离平地)
        if (-3.2 <= x <= 3.2) and (-3.2 <= y <= 3.2) and z >= self.ramp_enter_z:
            return True

        return False

    def update_loop(self):
        # 1. 查询小车当前位姿 (优先 TF，失败则无缝回退至里程计/真值缓存)
        x, y, z = None, None, None
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_base_frame, rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
        except TransformException:
            if self.last_pose is not None:
                x, y, z = self.last_pose
            else:
                return

        # 2. 判断所属区域
        in_ramp_box = self.is_in_ramp_or_platform_box(x, y, z)

        if z >= self.ground_z_thresh or in_ramp_box:
            detected_zone = ZONE_RAMP_PLATFORM
        elif z < self.ground_recover_z and not in_ramp_box:
            detected_zone = ZONE_GROUND
        else:
            # 处于过渡带，保持现有状态 (滞环特性)
            detected_zone = self.current_zone or ZONE_GROUND

        # 3. 防抖逻辑 (Debounce)
        if detected_zone != self.current_zone:
            if detected_zone == self.pending_zone:
                self.pending_count += 1
                if self.pending_count >= self.debounce_threshold:
                    self.apply_zone_switch(detected_zone, x, y, z)
                    self.pending_zone = None
                    self.pending_count = 0
            else:
                self.pending_zone = detected_zone
                self.pending_count = 1
        else:
            self.pending_zone = None
            self.pending_count = 0

    def apply_zone_switch(self, new_zone, x, y, z):
        old_zone = self.current_zone
        self.current_zone = new_zone

        if new_zone == ZONE_RAMP_PLATFORM:
            print("\n" + "=" * 70)
            print(
                f"\033[1;43;30m [ZoneAdaptive] >>> 切入【坡道与高台模式】 (RAMP_PLATFORM) \033[0m"
            )
            print(
                f"  当前位姿: X={x:+.2f}m, Y={y:+.2f}m, Z={z:+.2f}m\n"
                f"  \033[1;33m- MPPI 速度限幅: vx_max=0.9 m/s, vy_max=0.2 m/s, wz_max=1.5 rad/s (抑制横向侧滑与大幅甩尾)\033[0m\n"
                f"  \033[1;33m- MPPI 路径跟踪: PathAlignCritic=16.0, PathFollowCritic=12.0 (强对准中心线)\033[0m\n"
                f"  \033[1;33m- MPPI 避障容差: CostCritic=6.0 (适度容差窄坡走廊)\033[0m\n"
                f"  \033[1;33m- Costmap 膨胀: 缩减至 0.30m (cost_scaling_factor=6.0，打通 1 米窄坡走廊并形成居中引力谷)\033[0m"
            )
            print("=" * 70 + "\n")

            # 1. 下发 MPPI 参数 (窄坡高台稳健型)
            mppi_params = {
                "FollowPath.vx_max": 0.9,
                "FollowPath.vx_min": -0.9,
                "FollowPath.vy_max": 0.2,
                "FollowPath.wz_max": 1.5,
                "FollowPath.ax_max": 2.0,
                "FollowPath.ax_min": -2.0,
                "FollowPath.ay_max": 1.0,
                "FollowPath.ay_min": -1.0,
                "FollowPath.PathAlignCritic.cost_weight": 16.0,
                "FollowPath.PathFollowCritic.cost_weight": 12.0,
                "FollowPath.CostCritic.cost_weight": 6.0,
            }
            self._set_parameters_async(self.controller_server_name, mppi_params)

            # 2. 下发 Costmap 膨胀参数 (小膨胀，确保 1 米坡通过性)
            costmap_params = {
                "inflation_layer.inflation_radius": 0.30,
                "inflation_layer.cost_scaling_factor": 6.0,
            }
            self._set_parameters_async(self.local_costmap_name, costmap_params)
            self._set_parameters_async(self.global_costmap_name, costmap_params)

        else:
            print("\n" + "=" * 70)
            print(
                f"\033[1;42;37m [ZoneAdaptive] >>> 恢复【平地地面模式】 (GROUND) \033[0m"
            )
            print(
                f"  当前位姿: X={x:+.2f}m, Y={y:+.2f}m, Z={z:+.2f}m\n"
                f"  \033[1;32m- MPPI 速度限幅: vx_max=2.0 m/s, vy_max=1.8 m/s, wz_max=3.5 rad/s (全速全向移动)\033[0m\n"
                f"  \033[1;32m- MPPI 路径跟踪: PathAlignCritic=6.0, PathFollowCritic=6.0\033[0m\n"
                f"  \033[1;32m- MPPI 避障权重: CostCritic=12.0 (强避障排斥力)\033[0m\n"
                f"  \033[1;32m- Costmap 膨胀: 恢复为 0.48m (cost_scaling_factor=3.0，安全防贴墙)\033[0m"
            )
            print("=" * 70 + "\n")

            # 1. 下发 MPPI 参数 (平地高速机动型)
            mppi_params = {
                "FollowPath.vx_max": 2.0,
                "FollowPath.vx_min": -2.0,
                "FollowPath.vy_max": 1.8,
                "FollowPath.wz_max": 3.5,
                "FollowPath.ax_max": 3.5,
                "FollowPath.ax_min": -3.5,
                "FollowPath.ay_max": 3.0,
                "FollowPath.ay_min": -3.0,
                "FollowPath.PathAlignCritic.cost_weight": 6.0,
                "FollowPath.PathFollowCritic.cost_weight": 6.0,
                "FollowPath.CostCritic.cost_weight": 12.0,
            }
            self._set_parameters_async(self.controller_server_name, mppi_params)

            # 2. 下发 Costmap 膨胀参数 (大膨胀防贴墙)
            costmap_params = {
                "inflation_layer.inflation_radius": 0.48,
                "inflation_layer.cost_scaling_factor": 3.0,
            }
            self._set_parameters_async(self.local_costmap_name, costmap_params)
            self._set_parameters_async(self.global_costmap_name, costmap_params)

        self._publish_zone(new_zone)

    def _publish_zone(self, zone_str):
        msg = String()
        msg.data = zone_str
        self.zone_pub.publish(msg)


def main():
    rclpy.init()
    node = ZoneAdaptiveController()
    try:
        rclpy.spin(node)
    except Exception:
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


if __name__ == "__main__":
    main()
