# SimpleStairClimber Plugin

## 概述

SimpleStairClimber 是一个简化的爬楼梯插件，通过直接控制机器人整体位置（施加力）来实现爬楼梯和下楼梯功能。

与原始的 StairClimber 插件不同，SimpleStairClimber 不控制单个升降轮，而是直接对底盘施加力，使机器人整体移动。

## 设计理念

- **简单直接**: 不需要复杂的升降轮控制逻辑
- **力控制**: 使用 `AddWorldWrench()` 施加力来移动机器人
- **两阶段运动**: 
  - 爬升: 先向上，再向前
  - 下降: 先向后，再向下

## 系统参数

在 SDF 文件中配置插件时，需要指定以下参数：

```xml
<plugin filename="libSimpleStairClimber.so" name="ignition::gazebo::systems::SimpleStairClimber">
  <chassis_link>chassis</chassis_link>
</plugin>
```

### 参数说明

- `<chassis_link>`: 底盘 link 名称（必需）

## 订阅话题

### 1. 爬升指令

- **话题**: `/{model_name}/climb_stair`
- **消息类型**: `ignition.msgs.Double`
- **参数**: 台阶高度（米）

**示例**:
```bash
ign topic -t /AT_R2/climb_stair -m ignition.msgs.Double -p "data: 0.15"
```

### 2. 下降指令

- **话题**: `/{model_name}/descend_stair`
- **消息类型**: `ignition.msgs.Double`
- **参数**: 台阶高度（米）

**示例**:
```bash
ign topic -t /AT_R2/descend_stair -m ignition.msgs.Double -p "data: 0.15"
```

## 运动逻辑

### 爬升台阶

1. **阶段1 - 向上移动**: 
   - 施加向上的力 (5000N)
   - 移动距离 = 台阶高度
   - 容差: ±2cm

2. **阶段2 - 向前移动**:
   - 施加向前的力 (2000N)
   - 移动距离 = 0.5m
   - 容差: ±2cm

### 下降台阶

1. **阶段1 - 向后移动**:
   - 施加向后的力 (-2000N)
   - 移动距离 = 0.5m
   - 容差: ±2cm

2. **阶段2 - 向下移动**:
   - 施加向下的力 (-5000N)
   - 移动距离 = 台阶高度
   - 容差: ±2cm

## 状态机

```
IDLE (空闲)
  ↓ 收到爬升指令
CLIMB_UP (向上移动)
  ↓ 到达目标高度
CLIMB_FORWARD (向前移动)
  ↓ 到达目标距离
COMPLETE (完成) → IDLE

IDLE (空闲)
  ↓ 收到下降指令
DESCEND_BACKWARD (向后移动)
  ↓ 到达目标距离
DESCEND_DOWN (向下移动)
  ↓ 到达目标高度
COMPLETE (完成) → IDLE
```

## 配置参数

插件内部的可调参数（在代码中定义）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `HORIZONTAL_DISTANCE` | 0.5m | 水平移动距离 |
| `VERTICAL_FORCE` | 5000N | 垂直方向力 |
| `HORIZONTAL_FORCE` | 2000N | 水平方向力 |
| `POSITION_TOLERANCE` | 0.02m | 位置容差 |

## 使用示例

### 1. 在 SDF 文件中配置

```xml
<model name="AT_R2">
  <link name="chassis">
    <!-- ... -->
  </link>
  
  <plugin filename="libSimpleStairClimber.so" name="ignition::gazebo::systems::SimpleStairClimber">
    <chassis_link>chassis</chassis_link>
  </plugin>
</model>
```

### 2. 发送爬升指令

```bash
# 爬升 15cm 台阶
ign topic -t /AT_R2/climb_stair -m ignition.msgs.Double -p "data: 0.15"

# 爬升 20cm 台阶
ign topic -t /AT_R2/climb_stair -m ignition.msgs.Double -p "data: 0.20"
```

### 3. 发送下降指令

```bash
# 下降 15cm 台阶
ign topic -t /AT_R2/descend_stair -m ignition.msgs.Double -p "data: 0.15"

# 下降 20cm 台阶
ign topic -t /AT_R2/descend_stair -m ignition.msgs.Double -p "data: 0.20"
```

### 4. 使用测试脚本

```bash
cd rcu_ws
./test_simple_stair_climber.py AT_R2 0.15
```

## 与 StairClimber 的对比

| 特性 | StairClimber | SimpleStairClimber |
|------|--------------|-------------------|
| 控制方式 | 控制4个升降轮关节 | 直接对底盘施加力 |
| 复杂度 | 高（多阶段状态机） | 低（简单两阶段） |
| 状态数量 | 12个爬升状态 + 8个下降状态 | 5个状态 |
| 稳定性控制 | PID姿态稳定 | 无 |
| 超时保护 | 有（30秒） | 无 |
| 适用场景 | 真实物理仿真 | 简化测试/演示 |

## 注意事项

1. **物理仿真**: 此插件依赖于 Ignition Gazebo 的物理引擎，力的大小可能需要根据机器人质量调整

2. **碰撞检测**: 确保机器人模型有正确的碰撞体，否则可能穿透地面或台阶

3. **力的调整**: 如果机器人移动过快或过慢，可以调整 `VERTICAL_FORCE` 和 `HORIZONTAL_FORCE` 参数

4. **容差设置**: `POSITION_TOLERANCE` 决定了位置精度，过小可能导致无法完成，过大可能导致位置不准确

5. **并发控制**: 插件使用互斥锁保护指令，但不建议在运动过程中发送新指令

## 调试信息

插件会输出详细的日志信息：

```
SimpleStairClimber: Starting climb for 0.15m stair
SimpleStairClimber: Climb up complete (height: 0.151m), moving forward...
SimpleStairClimber: Climb forward complete (distance: 0.502m), climb finished!
SimpleStairClimber: ===== OPERATION COMPLETE! =====
```

## 编译

```bash
cd rcu_ws
colcon build --packages-select rmoss_gz_plugins
source install/setup.bash
```

## 故障排除

### 问题1: 机器人不移动

- 检查底盘 link 名称是否正确
- 检查是否正确加载了插件
- 查看终端日志是否有错误信息

### 问题2: 机器人移动过快/过慢

- 调整 `VERTICAL_FORCE` 和 `HORIZONTAL_FORCE` 参数
- 考虑机器人的质量和惯性

### 问题3: 机器人穿透地面

- 检查碰撞体设置
- 检查物理引擎参数
- 可能需要增加地面的摩擦系数

## 许可证

Apache License 2.0
