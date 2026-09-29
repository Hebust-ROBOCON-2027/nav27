"""RC2027 底盘串口桥接（R2 串口协议 v2.2 / 舵轮底盘）

用法:
  ros2 launch rc27_serial chassis_bridge.launch.py
  ros2 launch rc27_serial chassis_bridge.launch.py port_name:=/dev/ttyUSB0
  ros2 launch rc27_serial chassis_bridge.launch.py cmd_vel_topic:=/AT_R2/cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/ttyACM0",
            description="下位机设备节点：USB CDC 用 /dev/ttyACM0，UART 转串口用 /dev/ttyUSB0",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="协议规定 115200（UART TTL 3.3V, 8N1）",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/AT_R2/cmd_vel_nav2_result",
            description="订阅的速度话题；云台补偿后的用 /AT_R2/cmd_vel",
        ),
        DeclareLaunchArgument(
            "send_rate_hz",
            default_value="50",
            description="速度帧下发频率",
        ),
        DeclareLaunchArgument(
            "v_max_mps",
            default_value="3.0",
            description="合速度上限，超过按模长等比缩放（保持方向）",
        ),
        DeclareLaunchArgument(
            "w_max_radps",
            default_value="5.0",
            description="角速度上限",
        ),
    ]

    node = Node(
        package="rc27_serial",
        executable="chassis_bridge_node",
        name="chassis_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "port_name": LaunchConfiguration("port_name"),
                "baudrate": LaunchConfiguration("baudrate"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "send_rate_hz": LaunchConfiguration("send_rate_hz"),
                "v_max_mps": LaunchConfiguration("v_max_mps"),
                "w_max_radps": LaunchConfiguration("w_max_radps"),
            }
        ],
    )

    return LaunchDescription(args + [node])
