// Copyright 2024 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mutex>
#include <chrono>
#include <cmath>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/math/PID.hh>

#include "StairClimber.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// 爬楼梯状态机
enum class ClimbState
{
    IDLE,               // 空闲状态
    PHASE1_LIFT,        // 阶段1: 2号4号下降，抬起底盘
    PHASE2_FORWARD1,    // 阶段2: 前进，1号轮登上台阶
    PHASE2_STABILIZE,   // 阶段2稳定
    PHASE3_RETRACT2,    // 阶段3: 收起2号关节
    PHASE3_STABILIZE,   // 阶段3稳定
    PHASE4_FORWARD2,    // 阶段4: 前进，3号轮登上台阶
    PHASE4_STABILIZE,   // 阶段4稳定
    PHASE5_RETRACT4,    // 阶段5: 收起4号关节
    PHASE5_STABILIZE,   // 阶段5稳定
    PHASE6_FORWARD3,    // 阶段6: 最后前进一段
    COMPLETE            // 完成
};

/// 下降台阶状态机
enum class DescendState
{
    IDLE,                      // 空闲状态
    DESCEND_BACKWARD1,         // 阶段1: 倒退到后轮悬空
    DESCEND_STABILIZE1,        // 阶段1稳定
    DESCEND_LOWER_REAR,        // 阶段2: 伸出4号轮（后麦轮）
    DESCEND_STABILIZE2,        // 阶段2稳定
    DESCEND_BACKWARD2,         // 阶段3: 倒退到前轮边缘
    DESCEND_STABILIZE3,        // 阶段3稳定
    DESCEND_LOWER_MID,         // 阶段4: 伸出2号轮（中间升降轮）
    DESCEND_STABILIZE4,        // 阶段4稳定
    DESCEND_BACKWARD3,         // 阶段5: 倒退到完全下台阶
    DESCEND_STABILIZE5,        // 阶段5稳定
    DESCEND_RETRACT_ALL,       // 阶段6: 收起所有升降轮
    DESCEND_COMPLETE           // 完成
};

class ignition::gazebo::systems::StairClimberPrivate
{
public:
    void OnClimbCmd(const ignition::msgs::Double &_msg);
    void OnDescendCmd(const ignition::msgs::Double &_msg);
    void UpdateJointControl(const ignition::gazebo::UpdateInfo &_info,
                           ignition::gazebo::EntityComponentManager &_ecm);
    void UpdateDescendControl(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm);
    void AbortDescendSequence(ignition::gazebo::EntityComponentManager &_ecm);  // 安全中止下降序列
    void SetJointTarget(int jointIdx, double target);
    double GetJointPosition(int jointIdx, EntityComponentManager &_ecm);
    void ApplyForwardForce(EntityComponentManager &_ecm, double force);
    void ApplyBackwardForce(EntityComponentManager &_ecm, double force);
    void ApplyStabilizingTorque(EntityComponentManager &_ecm, const UpdateInfo &_info);

public:
    transport::Node node;
    Model model{kNullEntity};
    
    // 底盘 Link
    std::string chassisLinkName;
    Entity chassisLink{kNullEntity};
    
    // 4个升降关节
    // 0: front_lift_joint (1号 - 前麦轮, X=0.25)
    // 1: mid_lift_joint (2号 - 中间小轮, X=0.1)
    // 2: front_support_lift_joint (3号 - 前辅助轮, X=-0.12)
    // 3: rear_lift_joint (4号 - 后麦轮, X=-0.25)
    std::string jointNames[4];
    Entity joints[4];
    double jointTargets[4] = {0, 0, 0, 0};
    
    // 4个麦轮驱动关节
    Entity frontLeftWheel{kNullEntity};
    Entity frontRightWheel{kNullEntity};
    Entity rearLeftWheel{kNullEntity};
    Entity rearRightWheel{kNullEntity};
    
    // 关节限位
    double jointLowerLimits[4] = {-0.2, -0.25, -0.2, -0.2};
    double jointUpperLimits[4] = {0, 0, 0, 0};
    
    // 状态机
    ClimbState state = ClimbState::IDLE;
    double stairHeight = 0.0;
    std::chrono::steady_clock::time_point phaseStartTime;
    std::chrono::steady_clock::time_point stabilizeStartTime;
    
    // 下降台阶控制
    DescendState descend_state_ = DescendState::IDLE;
    double descend_start_x_ = 0.0;
    double descend_prev_x_ = 0.0;  // 上一次的X位置，用于检测意外前进
    std::chrono::steady_clock::time_point descend_sequence_start_time_;  // 整个下降序列的起始时间
    
    // 下降阈值参数（可配置）
    double rear_wheel_suspended_threshold_ = 0.20;   // 后轮悬空阈值 (m)
    double front_wheel_edge_threshold_ = 0.28;       // 前轮边缘阈值 (m)
    double descend_complete_threshold_ = 0.20;       // 下降完成阈值 (m)
    
    // 超时保护
    const double DESCEND_TIMEOUT_SEC = 30.0;  // 下降序列超时时间（秒）
    
    // 意外前进检测阈值
    const double UNEXPECTED_FORWARD_THRESHOLD = 0.01;  // 如果X增加超过1cm，视为意外前进
    
    // 前进控制
    double startX = 0.0;
    
    // 姿态稳定 PID
    ignition::math::PID pitchPid;
    ignition::math::PID rollPid;
    
    // 互斥锁
    std::mutex cmdMutex;
    bool newCommand = false;
    double pendingStairHeight = 0.0;
    
    // 下降指令控制
    bool newDescendCommand = false;
    double pendingDescendStairHeight = 0.0;
    
    // 稳定时间 (秒)
    const double STABILIZE_TIME = 0.8;
};

/******************implementation for StairClimber************************/
StairClimber::StairClimber() : dataPtr(std::make_unique<StairClimberPrivate>())
{
}

void StairClimber::Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "StairClimber plugin should be attached to a model entity." << std::endl;
        return;
    }
    
    // 获取底盘 Link
    this->dataPtr->chassisLinkName = _sdf->Get<std::string>("chassis_link", "chassis").first;
    this->dataPtr->chassisLink = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->chassisLinkName);
    if (this->dataPtr->chassisLink == kNullEntity)
    {
        ignerr << "Chassis link [" << this->dataPtr->chassisLinkName << "] not found." << std::endl;
        return;
    }
    
    // 创建必要的组件
    if (!_ecm.Component<components::WorldPose>(this->dataPtr->chassisLink))
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::WorldPose());
    if (!_ecm.Component<components::LinearVelocity>(this->dataPtr->chassisLink))
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::LinearVelocity());
    if (!_ecm.Component<components::AngularVelocity>(this->dataPtr->chassisLink))
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::AngularVelocity());
    
    // 获取关节名称
    this->dataPtr->jointNames[0] = _sdf->Get<std::string>("joint1", "front_lift_joint").first;
    this->dataPtr->jointNames[1] = _sdf->Get<std::string>("joint2", "mid_lift_joint").first;
    this->dataPtr->jointNames[2] = _sdf->Get<std::string>("joint3", "front_support_lift_joint").first;
    this->dataPtr->jointNames[3] = _sdf->Get<std::string>("joint4", "rear_lift_joint").first;
    
    // 获取关节实体并初始化
    for (int i = 0; i < 4; i++)
    {
        this->dataPtr->joints[i] = this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointNames[i]);
        if (this->dataPtr->joints[i] == kNullEntity)
        {
            ignerr << "Joint [" << this->dataPtr->jointNames[i] << "] not found." << std::endl;
            return;
        }
        
        if (!_ecm.Component<components::JointPosition>(this->dataPtr->joints[i]))
            _ecm.CreateComponent(this->dataPtr->joints[i], components::JointPosition());
    }
    
    // 获取麦轮驱动关节
    this->dataPtr->frontLeftWheel = this->dataPtr->model.JointByName(_ecm, "front_left_wheel_joint");
    this->dataPtr->frontRightWheel = this->dataPtr->model.JointByName(_ecm, "front_right_wheel_joint");
    this->dataPtr->rearLeftWheel = this->dataPtr->model.JointByName(_ecm, "rear_left_wheel_joint");
    this->dataPtr->rearRightWheel = this->dataPtr->model.JointByName(_ecm, "rear_right_wheel_joint");
    
    if (this->dataPtr->frontLeftWheel == kNullEntity || this->dataPtr->frontRightWheel == kNullEntity ||
        this->dataPtr->rearLeftWheel == kNullEntity || this->dataPtr->rearRightWheel == kNullEntity)
    {
        ignerr << "StairClimber: One or more wheel joints not found!" << std::endl;
        return;
    }
    
    // 姿态稳定 PID
    this->dataPtr->pitchPid.Init(500, 0, 100, 0, 0, 200, -200);
    this->dataPtr->rollPid.Init(500, 0, 100, 0, 0, 200, -200);
    
    // 订阅爬楼梯指令
    std::string topic = "/" + this->dataPtr->model.Name(_ecm) + "/climb_stair";
    this->dataPtr->node.Subscribe(topic, &StairClimberPrivate::OnClimbCmd, this->dataPtr.get());
    ignmsg << "StairClimber subscribing to [" << topic << "]" << std::endl;
    
    // 订阅下降台阶指令
    std::string descendTopic = "/" + this->dataPtr->model.Name(_ecm) + "/descend_stair";
    this->dataPtr->node.Subscribe(descendTopic, &StairClimberPrivate::OnDescendCmd, this->dataPtr.get());
    ignmsg << "StairClimber subscribing to [" << descendTopic << "]" << std::endl;
    
    ignmsg << "StairClimber initialized successfully" << std::endl;
}


void StairClimber::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;
    
    // 检查是否有新的爬升指令
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->cmdMutex);
        if (this->dataPtr->newCommand)
        {
            this->dataPtr->stairHeight = this->dataPtr->pendingStairHeight;
            this->dataPtr->newCommand = false;
            this->dataPtr->state = ClimbState::PHASE1_LIFT;
            this->dataPtr->phaseStartTime = std::chrono::steady_clock::now();
            
            auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->chassisLink)->Data();
            this->dataPtr->startX = pose.X();
            
            ignmsg << "StairClimber: Starting climb for " << this->dataPtr->stairHeight << "m stair" << std::endl;
        }
    }
    
    // 检查是否有新的下降指令
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->cmdMutex);
        if (this->dataPtr->newDescendCommand)
        {
            this->dataPtr->stairHeight = this->dataPtr->pendingDescendStairHeight;
            this->dataPtr->newDescendCommand = false;
            this->dataPtr->descend_state_ = DescendState::DESCEND_BACKWARD1;
            this->dataPtr->phaseStartTime = std::chrono::steady_clock::now();
            this->dataPtr->descend_sequence_start_time_ = std::chrono::steady_clock::now();  // 记录整个下降序列的起始时间
            
            auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->chassisLink)->Data();
            this->dataPtr->descend_start_x_ = pose.X();
            this->dataPtr->descend_prev_x_ = pose.X();  // 初始化上一次X位置
            
            ignmsg << "StairClimber: Starting descend for " << this->dataPtr->stairHeight << "m stair" << std::endl;
        }
    }
    
    // 调用爬升控制（仅当不在下降状态时）
    if (this->dataPtr->descend_state_ == DescendState::IDLE)
    {
        this->dataPtr->UpdateJointControl(_info, _ecm);
    }
    
    // 调用下降控制（仅当不在爬升状态时）
    if (this->dataPtr->state == ClimbState::IDLE && this->dataPtr->descend_state_ != DescendState::IDLE)
    {
        this->dataPtr->UpdateDescendControl(_info, _ecm);
    }
}

/******************implementation for StairClimberPrivate******************/

void StairClimberPrivate::OnClimbCmd(const ignition::msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    // if (_msg.data() <= 0.0)
    // {
    //     this->pendingDescendStairHeight = _msg.data();
    //     this->newDescendCommand = true;
    //     ignmsg << "StairClimber: Received descend command: " << _msg.data() << "m" << std::endl;
    // }
    //  else {
        this->pendingStairHeight = _msg.data();
        this->newCommand = true;
        ignmsg << "StairClimber: Received command: " << _msg.data() << "m" << std::endl;
    // }
}

void StairClimberPrivate::OnDescendCmd(const ignition::msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->pendingDescendStairHeight = _msg.data();
    this->newDescendCommand = true;
    ignmsg << "StairClimber: Received descend command: " << _msg.data() << "m" << std::endl;
}

double StairClimberPrivate::GetJointPosition(int jointIdx, EntityComponentManager &_ecm)
{
    auto posComp = _ecm.Component<components::JointPosition>(this->joints[jointIdx]);
    if (posComp && !posComp->Data().empty())
        return posComp->Data()[0];
    return 0.0;
}

void StairClimberPrivate::SetJointTarget(int jointIdx, double target)
{
    target = std::max(jointLowerLimits[jointIdx], std::min(jointUpperLimits[jointIdx], target));
    jointTargets[jointIdx] = target;
}

void StairClimberPrivate::ApplyForwardForce(EntityComponentManager &_ecm, double force)
{
    // 使用力控制驱动轮子
    Joint flWheel(frontLeftWheel);
    Joint frWheel(frontRightWheel);
    Joint rlWheel(rearLeftWheel);
    Joint rrWheel(rearRightWheel);
    
    std::vector<double> forces = {force};
    flWheel.SetForce(_ecm, forces);
    frWheel.SetForce(_ecm, forces);
    rlWheel.SetForce(_ecm, forces);
    rrWheel.SetForce(_ecm, forces);
}

void StairClimberPrivate::ApplyBackwardForce(EntityComponentManager &_ecm, double force)
{
    // 使用力控制驱动轮子倒退
    // force 参数应为负值（例如 -2.0）
    Joint flWheel(frontLeftWheel);
    Joint frWheel(frontRightWheel);
    Joint rlWheel(rearLeftWheel);
    Joint rrWheel(rearRightWheel);
    
    std::vector<double> forces = {force};
    flWheel.SetForce(_ecm, forces);
    frWheel.SetForce(_ecm, forces);
    rlWheel.SetForce(_ecm, forces);
    rrWheel.SetForce(_ecm, forces);
}

void StairClimberPrivate::ApplyStabilizingTorque(EntityComponentManager &_ecm, const UpdateInfo &_info)
{
    Link chassisLinkObj(this->chassisLink);
    auto pose = _ecm.Component<components::WorldPose>(this->chassisLink)->Data();
    
    // 获取当前俯仰角和横滚角
    double roll, pitch, yaw;
    pose.Rot().Euler(roll, pitch, yaw);
    
    // PID 控制使机器人保持水平
    double rollTorque = rollPid.Update(roll, _info.dt);
    double pitchTorque = pitchPid.Update(pitch, _info.dt);
    
    math::Vector3d torque(-rollTorque, -pitchTorque, 0);
    chassisLinkObj.AddWorldWrench(_ecm, math::Vector3d::Zero, torque);
}

void StairClimberPrivate::UpdateJointControl(const ignition::gazebo::UpdateInfo &_info,
                                            ignition::gazebo::EntityComponentManager &_ecm)
{
    auto now = std::chrono::steady_clock::now();
    double elapsedSec = std::chrono::duration<double>(now - phaseStartTime).count();
    double stabilizeElapsed = std::chrono::duration<double>(now - stabilizeStartTime).count();
    
    auto pose = _ecm.Component<components::WorldPose>(this->chassisLink)->Data();
    double currentX = pose.X();
    
    // 升降目标位置
    // 初始状态(关节=0)时轮子距底盘底部：
    // 4号后麦轮：10cm向下
    // 2号中间小轮：5cm向下
    // 4号比2号低5cm，所以要保持水平：2号需要比4号多伸出5cm
    
    double rearLiftAmount = -(stairHeight + 0.02);              // 4号：台阶高度+2cm余量
    double midLiftAmount = -(stairHeight + 0.02 + 0.05);        // 2号：多伸出5cm
    
    // 确保不超过关节限位
    if (midLiftAmount < -0.25) midLiftAmount = -0.25;
    if (rearLiftAmount < -0.2) rearLiftAmount = -0.2;
    
    switch (state)
    {
    case ClimbState::IDLE:
        for (int i = 0; i < 4; i++)
            SetJointTarget(i, 0);
        ApplyForwardForce(_ecm, 0.0);
        break;
        
    // ========== 阶段1: 抬起底盘 ==========
    case ClimbState::PHASE1_LIFT:
        SetJointTarget(0, 0);              // 1号保持
        SetJointTarget(1, midLiftAmount);  // 2号下降
        SetJointTarget(2, 0);              // 3号保持
        SetJointTarget(3, rearLiftAmount); // 4号下降
        
        if (elapsedSec > 2.0)
        {
            state = ClimbState::PHASE2_FORWARD1;
            phaseStartTime = now;
            startX = currentX;
            ignmsg << "StairClimber: Phase 1 lift done, moving forward..." << std::endl;
        }
        break;
        
    // ========== 阶段2: 前进让1号轮上台阶 ==========
    case ClimbState::PHASE2_FORWARD1:
        SetJointTarget(0, 0);
        SetJointTarget(1, midLiftAmount);
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount);
        ApplyForwardForce(_ecm, 2.0);
        ApplyStabilizingTorque(_ecm, _info);
        
        if ((currentX - startX) > 0.35 || elapsedSec > 5.0)
        {
            state = ClimbState::PHASE2_STABILIZE;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Phase 2 forward done, stabilizing..." << std::endl;
        }
        break;
        
    case ClimbState::PHASE2_STABILIZE:
        SetJointTarget(0, 0);
        SetJointTarget(1, midLiftAmount);
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount);
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            state = ClimbState::PHASE3_RETRACT2;
            phaseStartTime = now;
            ignmsg << "StairClimber: Retracting joint 2..." << std::endl;
        }
        break;
        
    // ========== 阶段3: 收起2号关节 ==========
    case ClimbState::PHASE3_RETRACT2:
        SetJointTarget(0, 0);
        SetJointTarget(1, 0);  // 2号收起
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount);  // 4号保持
        ApplyStabilizingTorque(_ecm, _info);
        
        if (elapsedSec > 1.5)
        {
            state = ClimbState::PHASE3_STABILIZE;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Joint 2 retracted, stabilizing..." << std::endl;
        }
        break;
        
    case ClimbState::PHASE3_STABILIZE:
        SetJointTarget(0, 0);
        SetJointTarget(1, 0);
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount);
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            state = ClimbState::PHASE4_FORWARD2;
            phaseStartTime = now;
            startX = currentX;
            ignmsg << "StairClimber: Moving forward (phase 4)..." << std::endl;
        }
        break;
        
    // ========== 阶段4: 前进让3号轮上台阶 ==========
    case ClimbState::PHASE4_FORWARD2:
        SetJointTarget(0, 0);
        SetJointTarget(1, 0);
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount);
        ApplyForwardForce(_ecm, 2.0);
        ApplyStabilizingTorque(_ecm, _info);
        
        if ((currentX - startX) > 0.28 || elapsedSec > 5.0)
        {
            state = ClimbState::PHASE4_STABILIZE;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Phase 4 forward done, stabilizing..." << std::endl;
        }
        break;
        
    case ClimbState::PHASE4_STABILIZE:
        SetJointTarget(0, 0);
        SetJointTarget(1, 0);
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount);
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            state = ClimbState::PHASE5_RETRACT4;
            phaseStartTime = now;
            ignmsg << "StairClimber: Retracting joint 4..." << std::endl;
        }
        break;
        
    // ========== 阶段5: 收起4号关节 ==========
    case ClimbState::PHASE5_RETRACT4:
        SetJointTarget(0, 0);
        SetJointTarget(1, 0);
        SetJointTarget(2, 0);
        SetJointTarget(3, 0);  // 4号收起
        ApplyStabilizingTorque(_ecm, _info);
        
        if (elapsedSec > 1.5)
        {
            state = ClimbState::PHASE5_STABILIZE;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Joint 4 retracted, stabilizing..." << std::endl;
        }
        break;
        
    case ClimbState::PHASE5_STABILIZE:
        for (int i = 0; i < 4; i++)
            SetJointTarget(i, 0);
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            state = ClimbState::PHASE6_FORWARD3;
            phaseStartTime = now;
            startX = currentX;
            ignmsg << "StairClimber: Final forward motion..." << std::endl;
        }
        break;
        
    // ========== 阶段6: 最后前进 ==========
    case ClimbState::PHASE6_FORWARD3:
        for (int i = 0; i < 4; i++)
            SetJointTarget(i, 0);
        ApplyForwardForce(_ecm, 2.0);
        
        if ((currentX - startX) > 0.15 || elapsedSec > 5.0)
        {
            state = ClimbState::COMPLETE;
            ignmsg << "StairClimber: ===== CLIMB COMPLETE! =====" << std::endl;
        }
        break;
        
    case ClimbState::COMPLETE:
        for (int i = 0; i < 4; i++)
            SetJointTarget(i, 0);
        state = ClimbState::IDLE;
        break;
    }
    
    // 应用关节力控制（使用固定力而不是PID）
    for (int i = 0; i < 4; i++)
    {
        double currentPos = GetJointPosition(i, _ecm);
        double target = jointTargets[i];
        double force = 0.0;
        
        // 根据目标位置决定施加的力
        if (std::abs(target) < 0.01)  // 目标是0（收回状态）
        {
            if (currentPos < -0.01)  // 当前伸出，需要收回
                force = 800.0;  // 向上拉力
            else
                force = 0.0;  // 已经收回，不施力
        }
        else  // 目标是伸出状态
        {
            if (currentPos > target + 0.01)  // 还没伸出到位
                force = -1200.0;  // 向下推力
            else
                force = -200.0;  // 保持力，防止回弹
        }
        
        Joint joint(joints[i]);
        std::vector<double> forces = {force};
        joint.SetForce(_ecm, forces);
    }
}

void StairClimberPrivate::UpdateDescendControl(const ignition::gazebo::UpdateInfo &_info,
                                              ignition::gazebo::EntityComponentManager &_ecm)
{
    auto now = std::chrono::steady_clock::now();
    double elapsedSec = std::chrono::duration<double>(now - phaseStartTime).count();
    double stabilizeElapsed = std::chrono::duration<double>(now - stabilizeStartTime).count();
    
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
    
    auto pose = _ecm.Component<components::WorldPose>(this->chassisLink)->Data();
    double currentX = pose.X();
    
    // 计算倒退距离（从起始点开始）
    double backwardDistance = descend_start_x_ - currentX;
    
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
    
    // 更新上一次的X位置
    descend_prev_x_ = currentX;
    
    // 升降目标位置（下降台阶）
    double rearLiftAmount = -(stairHeight + 0.02);              // 4号：台阶高度+2cm余量
    double midLiftAmount = -(stairHeight + 0.02 + 0.05);        // 2号：多伸出5cm
    
    // 确保不超过关节限位
    if (midLiftAmount < -0.25) midLiftAmount = -0.25;
    if (rearLiftAmount < -0.2) rearLiftAmount = -0.2;
    
    switch (descend_state_)
    {
    case DescendState::IDLE:
        // 空闲状态，不执行任何操作
        break;
        
    case DescendState::DESCEND_BACKWARD1:
        // 阶段1: 倒退到后轮悬空
        // 保持所有升降轮收起
        for (int i = 0; i < 4; i++)
            SetJointTarget(i, 0);
        
        // 施加倒退力，如果检测到意外前进则施加更强的纠正力
        if (unexpectedForward)
            ApplyBackwardForce(_ecm, -3.0);  // 纠正性倒退力
        else
            ApplyBackwardForce(_ecm, -2.0);  // 正常倒退力
        
        ApplyStabilizingTorque(_ecm, _info);
        
        // 监控倒退距离，当达到后轮悬空阈值时转换状态
        if (backwardDistance >= rear_wheel_suspended_threshold_ || elapsedSec > 10.0)
        {
            descend_state_ = DescendState::DESCEND_STABILIZE1;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Descend backward1 done (distance: " << backwardDistance 
                   << "m), stabilizing..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_STABILIZE1:
        // 稳定阶段1: 等待0.8秒
        for (int i = 0; i < 4; i++)
            SetJointTarget(i, 0);
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            descend_state_ = DescendState::DESCEND_LOWER_REAR;
            phaseStartTime = now;
            ignmsg << "StairClimber: Stabilize1 done, lowering rear wheel (joint 4)..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_LOWER_REAR:
        // 伸出4号轮（后麦轮）到 -(stairHeight + 0.02)
        SetJointTarget(0, 0);              // 1号保持
        SetJointTarget(1, 0);              // 2号保持
        SetJointTarget(2, 0);              // 3号保持
        SetJointTarget(3, rearLiftAmount); // 4号伸出
        ApplyStabilizingTorque(_ecm, _info);
        
        if (elapsedSec > 2.0)
        {
            descend_state_ = DescendState::DESCEND_STABILIZE2;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Rear wheel lowered, stabilizing..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_STABILIZE2:
        // 稳定阶段2: 等待0.8秒
        SetJointTarget(0, 0);
        SetJointTarget(1, 0);
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount); // 4号保持伸出
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            descend_state_ = DescendState::DESCEND_BACKWARD2;
            phaseStartTime = now;
            ignmsg << "StairClimber: Stabilize2 done, moving backward (phase 2)..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_BACKWARD2:
        // 阶段3: 倒退到前轮边缘
        // 保持4号轮伸出
        SetJointTarget(0, 0);              // 1号保持
        SetJointTarget(1, 0);              // 2号保持
        SetJointTarget(2, 0);              // 3号保持
        SetJointTarget(3, rearLiftAmount); // 4号保持伸出
        
        // 施加倒退力，如果检测到意外前进则施加更强的纠正力
        if (unexpectedForward)
            ApplyBackwardForce(_ecm, -3.0);  // 纠正性倒退力
        else
            ApplyBackwardForce(_ecm, -2.0);  // 正常倒退力
        
        ApplyStabilizingTorque(_ecm, _info);
        
        // 监控倒退距离，当达到前轮边缘阈值时转换状态
        if (backwardDistance >= front_wheel_edge_threshold_ || elapsedSec > 10.0)
        {
            descend_state_ = DescendState::DESCEND_STABILIZE3;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Descend backward2 done (distance: " << backwardDistance 
                   << "m), stabilizing..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_STABILIZE3:
        // 稳定阶段3: 等待0.8秒
        SetJointTarget(0, 0);
        SetJointTarget(1, 0);
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount); // 4号保持伸出
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            descend_state_ = DescendState::DESCEND_LOWER_MID;
            phaseStartTime = now;
            ignmsg << "StairClimber: Stabilize3 done, lowering mid wheel (joint 2)..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_LOWER_MID:
        // 伸出2号轮（中间升降轮）到 -(stairHeight + 0.07)
        SetJointTarget(0, 0);             // 1号保持
        SetJointTarget(1, midLiftAmount); // 2号伸出
        SetJointTarget(2, 0);             // 3号保持
        SetJointTarget(3, rearLiftAmount); // 4号保持伸出
        ApplyStabilizingTorque(_ecm, _info);
        
        if (elapsedSec > 2.0)
        {
            descend_state_ = DescendState::DESCEND_STABILIZE4;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Mid wheel lowered, stabilizing..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_STABILIZE4:
        // 稳定阶段4: 等待0.8秒
        SetJointTarget(0, 0);
        SetJointTarget(1, midLiftAmount);  // 2号保持伸出
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount); // 4号保持伸出
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            descend_state_ = DescendState::DESCEND_BACKWARD3;
            phaseStartTime = now;
            ignmsg << "StairClimber: Stabilize4 done, moving backward (phase 3)..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_BACKWARD3:
        // 阶段5: 倒退到完全下台阶
        // 保持2号和4号轮伸出
        SetJointTarget(0, 0);              // 1号保持
        SetJointTarget(1, midLiftAmount);  // 2号保持伸出
        SetJointTarget(2, 0);              // 3号保持
        SetJointTarget(3, rearLiftAmount); // 4号保持伸出
        
        // 施加倒退力，如果检测到意外前进则施加更强的纠正力
        if (unexpectedForward)
            ApplyBackwardForce(_ecm, -3.0);  // 纠正性倒退力
        else
            ApplyBackwardForce(_ecm, -2.0);  // 正常倒退力
        
        ApplyStabilizingTorque(_ecm, _info);
        
        // 监控倒退距离，当达到下降完成阈值时转换状态
        if (backwardDistance >= descend_complete_threshold_ || elapsedSec > 10.0)
        {
            descend_state_ = DescendState::DESCEND_STABILIZE5;
            stabilizeStartTime = now;
            ignmsg << "StairClimber: Descend backward3 done (distance: " << backwardDistance 
                   << "m), stabilizing..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_STABILIZE5:
        // 稳定阶段5: 等待0.8秒
        SetJointTarget(0, 0);
        SetJointTarget(1, midLiftAmount);  // 2号保持伸出
        SetJointTarget(2, 0);
        SetJointTarget(3, rearLiftAmount); // 4号保持伸出
        ApplyStabilizingTorque(_ecm, _info);
        
        if (stabilizeElapsed > STABILIZE_TIME)
        {
            descend_state_ = DescendState::DESCEND_RETRACT_ALL;
            phaseStartTime = now;
            ignmsg << "StairClimber: Stabilize5 done, retracting all lift joints..." << std::endl;
        }
        break;
        
    case DescendState::DESCEND_RETRACT_ALL:
        // 收起所有升降轮
        SetJointTarget(0, 0);  // 1号收起
        SetJointTarget(1, 0);  // 2号收起
        SetJointTarget(2, 0);  // 3号收起
        SetJointTarget(3, 0);  // 4号收起
        ApplyStabilizingTorque(_ecm, _info);
        
        // 等待1.5秒确保所有轮子收回
        if (elapsedSec > 1.5)
        {
            descend_state_ = DescendState::DESCEND_COMPLETE;
            ignmsg << "StairClimber: All lift joints retracted, descend complete!" << std::endl;
        }
        break;
        
    case DescendState::DESCEND_COMPLETE:
        // 完成状态：记录日志并重置为IDLE
        for (int i = 0; i < 4; i++)
            SetJointTarget(i, 0);
        
        ignmsg << "StairClimber: ===== DESCEND COMPLETE! =====" << std::endl;
        descend_state_ = DescendState::IDLE;
        break;
    }
    
    // 应用关节力控制（与上升台阶相同的逻辑）
    for (int i = 0; i < 4; i++)
    {
        double currentPos = GetJointPosition(i, _ecm);
        double target = jointTargets[i];
        double force = 0.0;
        
        // 根据目标位置决定施加的力
        if (std::abs(target) < 0.01)  // 目标是0（收回状态）
        {
            if (currentPos < -0.01)  // 当前伸出，需要收回
                force = 800.0;  // 向上拉力
            else
                force = 0.0;  // 已经收回，不施力
        }
        else  // 目标是伸出状态
        {
            if (currentPos > target + 0.01)  // 还没伸出到位
                force = -1200.0;  // 向下推力
            else
                force = -200.0;  // 保持力，防止回弹
        }
        
        Joint joint(joints[i]);
        std::vector<double> forces = {force};
        joint.SetForce(_ecm, forces);
    }
}

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

/******************register*************************************************/
IGNITION_ADD_PLUGIN(StairClimber,
                    ignition::gazebo::System,
                    StairClimber::ISystemConfigure,
                    StairClimber::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(StairClimber, "ignition::gazebo::systems::StairClimber")
