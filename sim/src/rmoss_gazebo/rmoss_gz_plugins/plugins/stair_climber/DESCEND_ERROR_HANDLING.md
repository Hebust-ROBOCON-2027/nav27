# 下降台阶错误处理实现文档

## 概述

本文档描述了下降台阶控制系统中实现的错误处理和超时保护机制。

## 实现的功能

### 1. 超时检测 (Task 7.1) ✓

**实现位置**: `UpdateDescendControl()` 方法

**功能描述**:
- 在整个下降序列开始时记录起始时间 (`descend_sequence_start_time_`)
- 在每次更新时检查经过的总时间
- 如果超过 30 秒 (`DESCEND_TIMEOUT_SEC`)，调用 `AbortDescendSequence()` 安全中止

**代码片段**:
```cpp
// 检查整个下降序列的超时（30秒）
if (descend_state_ != DescendState::IDLE && descend_state_ != DescendState::DESCEND_COMPLETE)
{
    double totalElapsed = std::chrono::duration<double>(now - descend_sequence_start_time_).count();
    if (totalElapsed > DESCEND_TIMEOUT_SEC)
    {
        ignerr << "StairClimber: Descend sequence TIMEOUT after " << totalElapsed 
               << " seconds! Aborting safely..." << std::endl;
        AbortDescendSequence(_ecm);
        return;
    }
}
```

### 2. 意外前进检测 (Task 7.2) ✓

**实现位置**: `UpdateDescendControl()` 方法

**功能描述**:
- 跟踪机器人上一次的 X 位置 (`descend_prev_x_`)
- 在倒退阶段 (DESCEND_BACKWARD1/2/3) 检测 X 位置的变化
- 如果 X 增加超过阈值 (1cm)，视为意外前进
- 记录警告日志并设置标志 (`unexpectedForward`)
- 在倒退状态中施加更强的纠正性倒退力 (-3.0 N 而非 -2.0 N)

**代码片段**:
```cpp
// 检测意外前进（仅在倒退阶段检查）
bool unexpectedForward = false;
if (descend_state_ == DescendState::DESCEND_BACKWARD1 ||
    descend_state_ == DescendState::DESCEND_BACKWARD2 ||
    descend_state_ == DescendState::DESCEND_BACKWARD3)
{
    double deltaX = currentX - descend_prev_x_;
    if (deltaX > UNEXPECTED_FORWARD_THRESHOLD)
    {
        ignwarn << "StairClimber: WARNING - Unexpected forward movement detected! "
                << "Delta X: " << deltaX << "m. Applying corrective backward force." << std::endl;
        unexpectedForward = true;
    }
}

// 在倒退状态中应用纠正力
if (unexpectedForward)
    ApplyBackwardForce(_ecm, -3.0);  // 纠正性倒退力
else
    ApplyBackwardForce(_ecm, -2.0);  // 正常倒退力
```

### 3. 安全中止机制 (Task 7.3) ✓

**实现位置**: `AbortDescendSequence()` 方法

**功能描述**:
- 停止所有运动（设置倒退力为 0）
- 收回所有升降轮到安全位置（目标设为 0）
- 应用向上拉力 (800.0 N) 收回伸出的升降轮
- 重置状态为 IDLE
- 记录错误日志

**代码片段**:
```cpp
void StairClimberPrivate::AbortDescendSequence(ignition::gazebo::EntityComponentManager &_ecm)
{
    ignerr << "StairClimber: Aborting descend sequence - ensuring safe position..." << std::endl;
    
    // 停止所有运动
    ApplyBackwardForce(_ecm, 0.0);
    
    // 收回所有升降轮到安全位置
    for (int i = 0; i < 4; i++)
    {
        SetJointTarget(i, 0);
    }
    
    // 应用关节力控制以收回升降轮
    for (int i = 0; i < 4; i++)
    {
        double currentPos = GetJointPosition(i, _ecm);
        double force = 0.0;
        
        if (currentPos < -0.01)  // 当前伸出，需要收回
            force = 800.0;  // 向上拉力
        
        Joint joint(joints[i]);
        std::vector<double> forces = {force};
        joint.SetForce(_ecm, forces);
    }
    
    // 重置状态为IDLE
    descend_state_ = DescendState::IDLE;
    
    ignerr << "StairClimber: Descend sequence aborted, returned to IDLE state" << std::endl;
}
```

## 配置参数

### 超时参数
- `DESCEND_TIMEOUT_SEC`: 30.0 秒（整个下降序列的最大允许时间）

### 意外前进检测参数
- `UNEXPECTED_FORWARD_THRESHOLD`: 0.01 米（1cm）
  - 如果机器人在倒退阶段向前移动超过此阈值，触发警告和纠正

### 纠正力参数
- 正常倒退力: -2.0 N
- 纠正性倒退力: -3.0 N（比正常力强 50%）

## 测试方法

### 手动测试
1. 启动仿真环境
2. 运行测试脚本: `./test/scripts/test_descend_stair.sh [robot_name]`
3. 观察 Gazebo 控制台日志

### 预期行为
- **正常情况**: 机器人平稳倒退，完成下降序列
- **意外前进**: 控制台显示 WARNING 消息，机器人施加纠正力
- **超时**: 30秒后自动中止，收回升降轮，返回 IDLE 状态

## 需求映射

| 任务 | 需求 | 状态 |
|-----|------|------|
| 7.1 超时检测 | 5.2 | ✓ 完成 |
| 7.2 意外前进检测 | 5.1 | ✓ 完成 |
| 7.3 安全中止机制 | 5.4 | ✓ 完成 |

## 日志消息

### 正常操作
- `"StairClimber: Received descend command: X.Xm"`
- `"StairClimber: Starting descend for X.Xm stair"`
- `"StairClimber: Descend backwardX done (distance: X.XXm), stabilizing..."`
- `"StairClimber: ===== DESCEND COMPLETE! ====="`

### 错误情况
- `"StairClimber: WARNING - Unexpected forward movement detected! Delta X: X.XXm. Applying corrective backward force."`
- `"StairClimber: Descend sequence TIMEOUT after X.XX seconds! Aborting safely..."`
- `"StairClimber: Aborting descend sequence - ensuring safe position..."`
- `"StairClimber: Descend sequence aborted, returned to IDLE state"`

## 未来改进建议

1. **参数可配置化**: 将超时时间和阈值作为 ROS2 参数，允许运行时调整
2. **更智能的纠正**: 根据前进速度动态调整纠正力大小
3. **状态恢复**: 在某些情况下允许从中止状态恢复，而不是完全重置
4. **遥测数据**: 记录更详细的遥测数据用于离线分析
