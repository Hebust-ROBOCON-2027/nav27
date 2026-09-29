第 1 步：只起桥接，确认串口通了（车不动）

bash
source install/setup.bash
ros2 launch rc27_serial chassis_bridge.launch.py
看日志：

code
串口已打开: /dev/ttyACM0 @ 115200
链路 已连接 | 发 0 收 0 | ...
发 0 是正常的——因为还没有 cmd_vel。这一步只验证串口通不通，车不会动。

第 2 步：手动点动，确认链路闭环（低速！）

bash
# 另一个终端，20Hz 持续发 0.1 m/s（必须持续发，节点 200ms 收不到就归零）
ros2 topic pub -r 20 /AT_R2/cmd_vel_nav2_result geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
此时应该看到：

发 计数在涨（50/秒）
收 计数在涨 → 说明电控在回 0x10 舵轮状态，双向通了
轮子开始转
Ctrl+C 停掉 pub，2 秒后节点自动归零并停发。

第 3 步：接导航，自动跑

bash
# 终端1：导航（实车！仿真用 ./run_navigation.sh）
ros2 launch at_r2_nav_bringup at_navigation_launch_red.py use_sim_time:=false

# 终端2：桥接
ros2 launch rc27_serial chassis_bridge.launch.py
然后在 RViz 里给目标点，Nav2 就会持续输出 /AT_R2/cmd_vel_nav2_result，车跟着走。

首次上电的安全建议
bash
# 把限速压到 0.3 m/s，确认方向对了再放开
ros2 launch rc27_serial chassis_bridge.launch.py v_max_mps:=0.3 w_max_radps:=1.0

# 或者超时直接发急停帧（默认只发零速）
ros2 launch rc27_serial chassis_bridge.launch.py estop_on_timeout:=true
先把车架起来（轮子离地）跑第 2 步，确认转向、方向没问题再落地
方向反了改参数，不用改代码：invert_vx:=true / invert_wz:=true / swap_xy:=true
随时能停：Ctrl+C 掉 pub 或导航，节点 200ms 内自动发零速
如果 废帧 CRC 一直涨 → CRC 覆盖范围两边不一致，回去对 CRC_START_OFFSET