# rc27_serial

RC2027 上位机 ↔ 下位机串口通信包，实现 **R2 协议 v2.2（舵轮底盘）**。

> **协议文档见 [`PROTOCOL.md`](PROTOCOL.md)**，发给电控看那份。
> 协议真源是 `include/rc27_serial/protocol.hpp`，改字段先改头文件。

---

## 它做什么

订阅 Nav2 的 `cmd_vel`，按 50 Hz 把车体速度（`vx/vy/wz`）打包成串口帧发给电控；
同时接收电控回传的舵轮状态（4 个转向角 + 4 个轮速），发布到 ROS 话题。

```
Nav2 cmd_vel ──> chassis_bridge_node ──> 串口帧 0x01 ──> 电控（舵轮解算）
                        ↑                                    │
                        └──── 串口帧 0x10（舵轮状态）────────┘
                                    ↓
                          /AT_R2/steer_state
```

---

## 运行       ls /dev/ttyACM0

```bash
cd /home/hao/nav27/nav && colcon build --packages-select rc27_serial
source install/setup.bash

ros2 launch rc27_serial chassis_bridge.launch.py
ros2 launch rc27_serial chassis_bridge.launch.py port_name:=/dev/ttyUSB0
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `port_name` | `/dev/ttyACM0` | USB CDC 默认节点；若是 UART 转串口（CH340 等）改 `/dev/ttyUSB0` |
| `baudrate` | `115200` | CDC 下被内核忽略，仅物理 UART 时生效 |
| `cmd_vel_topic` | `/AT_R2/cmd_vel_nav2_result` | 订阅的速度话题（云台补偿后的用 `/AT_R2/cmd_vel`） |
| `send_rate_hz` | `50` | 速度帧下发频率 |
| `cmd_vel_timeout_ms` | `200` | 超时后改发零速 |
| `stop_repeat_n` | `10` | 零速重复次数，之后停发 |
| `v_max_mps` | `3.0` | 合速度上限（按模长等比缩放，保持方向） |
| `w_max_radps` | `5.0` | 角速度上限 |
| `estop_on_timeout` | `false` | 超时改发 `0x02` 急停帧（默认只发零速） |
| `invert_vx/vy/wz` | `false` | 方向取反，装机反了改参数，不用改代码 |
| `swap_xy` | `false` | 交换 vx / vy |

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/AT_R2/cmd_vel_nav2_result` | `geometry_msgs/Twist` | 订阅（可配） |
| `/AT_R2/steer_state` | `std_msgs/Float32MultiArray` | 发布，前 4 个转向角(rad)，后 4 个轮速(m/s) |

## 日志怎么看

节点每 2 秒打一条链路状态：

```
链路 已连接 | 发 1234 收 1200 | 发失败 0 重连 0 | 废帧 头0 长0 CRC0
```

- `链路 断开` → 串口没打开，查设备名 / 权限（udev）
- `废帧 CRC` 一直涨 → CRC 覆盖范围或波特率两边不一致
- `废帧 长` 涨 → 数据区长度对不上，查 `LEN` 定义

---

## 代码结构

| 文件 | 职责 |
|---|---|
| `protocol.hpp` | **协议真源**：帧常量、命令字、数据区、CRC16 |
| `ring_parser.hpp` | 流式解析：环形缓冲 + 帧头/LEN/CRC 三重校验 |
| `serial_port.hpp/cpp` | 串口 I/O：组帧、收发、断线重连（不含业务） |
| `chassis_interface.hpp` | `IChassis` 纯虚接口（上层只依赖接口） |
| `serial_chassis.hpp/cpp` | `IChassis` 的串口实现（`setVelocity` / `emergencyStop` / `getSteerState`） |
| `chassis_bridge_node.cpp` | ROS 节点：订阅 cmd_vel → `IChassis` |

换通信方式（CAN、仿真）时，新写一个 `IChassis` 实现类，
只改节点里创建对象的那一行，其余代码不动。
