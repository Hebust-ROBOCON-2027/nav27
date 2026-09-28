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
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/int32.pb.h>

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include "SimpleStairClimber.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// 简化爬楼梯状态机
enum class SimpleClimbState
{
    IDLE = 0,            // 空闲状态
    CLIMB_UP = 1,        // 上楼梯：向上移动
    CLIMB_FORWARD = 2,   // 上楼梯：向前移动
    DESCEND_BACKWARD = 3,// 下楼梯：向后移动
    DESCEND_DOWN = 4,    // 下楼梯：向下移动
    COMPLETE = 5         // 完成
};

/// 状态码定义（用于外部通信）
enum class ClimberStatus
{
    IDLE = 0,        // 未执行/空闲
    RUNNING = 1,     // 执行中
    SUCCESS = 2,     // 执行完毕/成功
    FAILED = 3       // 执行失败
};

class ignition::gazebo::systems::SimpleStairClimberPrivate
{
public:
    void OnClimbCmd(const ignition::msgs::Double &_msg);
    void OnDescendCmd(const ignition::msgs::Double &_msg);
    void UpdateControl(const ignition::gazebo::UpdateInfo &_info,
                      ignition::gazebo::EntityComponentManager &_ecm);
    void ApplyForce(EntityComponentManager &_ecm, const math::Vector3d &force);
    void ApplyTorque(EntityComponentManager &_ecm, const math::Vector3d &torque);
    void PublishStatus(ClimberStatus status, bool force = false);
    
    // 坐标转换辅助函数
    math::Vector3d TransformForceToWorld(const math::Vector3d &robotFrameForce, double yaw);
    math::Vector3d TransformTorqueToWorld(const math::Vector3d &robotFrameTorque, double yaw);
    void GetRobotOrientation(const math::Pose3d &pose, double &roll, double &pitch, double &yaw);
    double GetRobotYaw(const math::Pose3d &pose);

public:
    transport::Node node;
    Model model{kNullEntity};
    
    // 底盘 Link
    std::string chassisLinkName;
    Entity chassisLink{kNullEntity};
    
    // 状态发布器
    transport::Node::Publisher statusPub;
    
    // 状态机
    SimpleClimbState state = SimpleClimbState::IDLE;
    ClimberStatus lastPublishedStatus = ClimberStatus::IDLE;
    double targetHeight = 0.0;
    double startZ = 0.0;
    double startX = 0.0;
    double startY = 0.0;  // 添加Y坐标记录
    double startYaw = 0.0;  // 添加yaw角度记录，用于水平移动阶段
    std::chrono::steady_clock::time_point lastStatusPublishTime;
    
    // 运动参数
    const double HORIZONTAL_DISTANCE = 1.0;  // 水平移动距离 (m)
    const double ROBOT_MASS = 12.0;          // 机器人总质量 (kg) - 现在只有底盘有质量
    const double GRAVITY = 9.81;             // 重力加速度 (m/s²)
    const double GRAVITY_COMPENSATION = 120.0;  // 重力补偿力 (N) - 约等于底盘重力
    const double HEIGHT_HOLD_KP = 500.0;     // 高度保持比例系数
    const double HEIGHT_HOLD_KD = 150.0;     // 高度保持微分系数
    const double MAX_HEIGHT_FORCE = 1000.0;  // 最大高度保持力 (N)
    const double ANGLE_STABILIZE_KP = 500.0; // 角度稳定比例系数
    const double ANGLE_STABILIZE_KD = 150.0; // 角度稳定微分系数
    const double MAX_TORQUE = 50.0;          // 最大稳定力矩 (N·m) - 从100降低到50
    const double VERTICAL_FORCE = 800.0;     // 垂直方向力 (N)
    const double HORIZONTAL_FORCE = 400.0;   // 水平方向力 (N)
    const double MAX_VERTICAL_VELOCITY = 0.5;   // 最大垂直速度 (m/s)
    const double MAX_HORIZONTAL_VELOCITY = 0.6; // 最大水平速度 (m/s)
    const double VELOCITY_DAMPING = 150.0;   // 速度阻尼系数
    const double POSITION_TOLERANCE = 0.02;  // 位置容差 (m)
    
    // 互斥锁
    std::mutex cmdMutex;
    bool newClimbCommand = false;
    bool newDescendCommand = false;
    double pendingStairHeight = 0.0;
};

/******************implementation for SimpleStairClimber************************/
SimpleStairClimber::SimpleStairClimber() : dataPtr(std::make_unique<SimpleStairClimberPrivate>())
{
}

void SimpleStairClimber::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "SimpleStairClimber plugin should be attached to a model entity." << std::endl;
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
    
    // 订阅爬楼梯指令
    std::string climbTopic = "/" + this->dataPtr->model.Name(_ecm) + "/climb_stair";
    this->dataPtr->node.Subscribe(climbTopic, &SimpleStairClimberPrivate::OnClimbCmd, this->dataPtr.get());
    ignmsg << "SimpleStairClimber subscribing to [" << climbTopic << "]" << std::endl;
    
    // 订阅下楼梯指令
    std::string descendTopic = "/" + this->dataPtr->model.Name(_ecm) + "/descend_stair";
    this->dataPtr->node.Subscribe(descendTopic, &SimpleStairClimberPrivate::OnDescendCmd, this->dataPtr.get());
    ignmsg << "SimpleStairClimber subscribing to [" << descendTopic << "]" << std::endl;
    
    // 创建状态发布器
    std::string statusTopic = "/" + this->dataPtr->model.Name(_ecm) + "/climber_status";
    this->dataPtr->statusPub = this->dataPtr->node.Advertise<ignition::msgs::Int32>(statusTopic);
    ignmsg << "SimpleStairClimber publishing status to [" << statusTopic << "]" << std::endl;
    
    // 初始化状态发布时间
    this->dataPtr->lastStatusPublishTime = std::chrono::steady_clock::now();
    
    // 发布初始状态（IDLE），强制发布
    this->dataPtr->PublishStatus(ClimberStatus::IDLE, true);
    
    ignmsg << "SimpleStairClimber initialized successfully" << std::endl;
}

void SimpleStairClimber::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;
    
    // 检查是否有新的爬升指令
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->cmdMutex);
        if (this->dataPtr->newClimbCommand)
        {
            this->dataPtr->targetHeight = this->dataPtr->pendingStairHeight;
            this->dataPtr->newClimbCommand = false;
            this->dataPtr->state = SimpleClimbState::CLIMB_UP;
            
            auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->chassisLink)->Data();
            this->dataPtr->startZ = pose.Z();
            this->dataPtr->startX = pose.X();
            this->dataPtr->startY = pose.Y();  // 保存初始Y坐标
            
            // 发布状态：开始执行
            this->dataPtr->PublishStatus(ClimberStatus::RUNNING);
            
            ignmsg << "SimpleStairClimber: Starting climb for " << this->dataPtr->targetHeight << "m stair" << std::endl;
        }
    }
    
    // 检查是否有新的下降指令
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->cmdMutex);
        if (this->dataPtr->newDescendCommand)
        {
            this->dataPtr->targetHeight = this->dataPtr->pendingStairHeight;
            this->dataPtr->newDescendCommand = false;
            this->dataPtr->state = SimpleClimbState::DESCEND_BACKWARD;
            
            auto pose = _ecm.Component<components::WorldPose>(this->dataPtr->chassisLink)->Data();
            this->dataPtr->startZ = pose.Z();
            this->dataPtr->startX = pose.X();
            this->dataPtr->startY = pose.Y();  // 保存初始Y坐标
            this->dataPtr->startYaw = this->dataPtr->GetRobotYaw(pose);  // 保存初始yaw角度
            
            // 发布状态：开始执行
            this->dataPtr->PublishStatus(ClimberStatus::RUNNING);
            
            ignmsg << "SimpleStairClimber: Starting descend for " << this->dataPtr->targetHeight << "m stair" << std::endl;
        }
    }
    
    // 更新控制
    this->dataPtr->UpdateControl(_info, _ecm);
}

/******************implementation for SimpleStairClimberPrivate******************/

void SimpleStairClimberPrivate::OnClimbCmd(const ignition::msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->pendingStairHeight = _msg.data();
    this->newClimbCommand = true;
    ignmsg << "SimpleStairClimber: Received climb command: " << _msg.data() << "m" << std::endl;
}

void SimpleStairClimberPrivate::PublishStatus(ClimberStatus status, bool force)
{
    // 只在状态改变时发布，或者强制发布
    if (force || status != lastPublishedStatus)
    {
        ignition::msgs::Int32 msg;
        msg.set_data(static_cast<int32_t>(status));
        this->statusPub.Publish(msg);
        lastPublishedStatus = status;
        lastStatusPublishTime = std::chrono::steady_clock::now();
        
        // 打印状态变化日志
        const char* status_names[] = {"IDLE", "RUNNING", "SUCCESS", "FAILED"};
        ignmsg << "SimpleStairClimber: Status changed to " 
               << status_names[static_cast<int>(status)] << std::endl;
    }
}

void SimpleStairClimberPrivate::OnDescendCmd(const ignition::msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->pendingStairHeight = _msg.data();
    this->newDescendCommand = true;
    ignmsg << "SimpleStairClimber: Received descend command: " << _msg.data() << "m" << std::endl;
}

math::Vector3d SimpleStairClimberPrivate::TransformForceToWorld(
    const math::Vector3d &robotFrameForce,
    double yaw)
{
    // 使用显式的cos/sin公式进行旋转变换
    // 机器人坐标系 → 世界坐标系
    // world_x = robot_x * cos(yaw) - robot_y * sin(yaw)
    // world_y = robot_x * sin(yaw) + robot_y * cos(yaw)
    // world_z = robot_z
    
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    math::Vector3d worldForce(
        robotFrameForce.X() * cos_yaw - robotFrameForce.Y() * sin_yaw,
        robotFrameForce.X() * sin_yaw + robotFrameForce.Y() * cos_yaw,
        robotFrameForce.Z()
    );
    
    return worldForce;
}

math::Vector3d SimpleStairClimberPrivate::TransformTorqueToWorld(
    const math::Vector3d &robotFrameTorque,
    double yaw)
{
    // 力矩的转换与力的转换相同（都是向量的旋转变换）
    // 机器人坐标系 → 世界坐标系
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    math::Vector3d worldTorque(
        robotFrameTorque.X() * cos_yaw - robotFrameTorque.Y() * sin_yaw,
        robotFrameTorque.X() * sin_yaw + robotFrameTorque.Y() * cos_yaw,
        robotFrameTorque.Z()
    );
    
    return worldTorque;
}

void SimpleStairClimberPrivate::GetRobotOrientation(
    const math::Pose3d &pose,
    double &roll,
    double &pitch,
    double &yaw)
{
    // 获取世界坐标系下的四元数
    auto quat = pose.Rot();
    
    // 提取yaw角度（绕世界Z轴的旋转）
    yaw = quat.Yaw();
    
    // 计算机器人本体坐标系相对于重力方向的roll和pitch
    // 方法：将世界坐标系的Z轴（重力方向）转换到机器人本体坐标系
    // 然后从这个向量中提取roll和pitch
    
    // 世界坐标系的Z轴单位向量
    math::Vector3d worldZ(0, 0, 1);
    
    // 将世界Z轴转换到机器人本体坐标系
    // 使用四元数的逆旋转
    math::Vector3d robotZ = quat.RotateVectorReverse(worldZ);
    
    // 从机器人坐标系中的重力方向向量计算roll和pitch
    // robotZ.X() 对应 pitch（前后倾斜）
    // robotZ.Y() 对应 roll（左右倾斜）
    // robotZ.Z() 应该接近1（如果机器人水平）
    
    // pitch: 机器人前后倾斜角度
    // 当机器人前倾时，重力在机器人X轴上有正分量
    pitch = std::asin(std::clamp(robotZ.X(), -1.0, 1.0));
    
    // roll: 机器人左右倾斜角度
    // 当机器人右倾时，重力在机器人Y轴上有负分量
    roll = std::atan2(-robotZ.Y(), robotZ.Z());
}

double SimpleStairClimberPrivate::GetRobotYaw(const math::Pose3d &pose)
{
    // 从姿态中提取yaw角度（绕世界Z轴的旋转）
    return pose.Rot().Yaw();
}

void SimpleStairClimberPrivate::ApplyForce(EntityComponentManager &_ecm, const math::Vector3d &force)
{
    Link chassisLink(this->chassisLink);
    
    // 在世界坐标系中施加力
    // 注意：由于所有轮子和支架都通过fixed joint连接到chassis，
    // 施加在chassis上的力会影响整个刚体系统
    chassisLink.AddWorldWrench(_ecm, force, math::Vector3d::Zero);
}

void SimpleStairClimberPrivate::ApplyTorque(EntityComponentManager &_ecm, const math::Vector3d &torque)
{
    Link chassisLink(this->chassisLink);
    
    // 获取当前姿态
    auto pose = _ecm.Component<components::WorldPose>(this->chassisLink)->Data();
    auto quat = pose.Rot();
    
    // 将本体坐标系的力矩转换到世界坐标系
    // 使用四元数旋转向量
    math::Vector3d worldTorque = quat.RotateVector(torque);
    
    // 在世界坐标系中施加力矩
    chassisLink.AddWorldWrench(_ecm, math::Vector3d::Zero, worldTorque);
}

void SimpleStairClimberPrivate::UpdateControl(const ignition::gazebo::UpdateInfo &_info,
                                             ignition::gazebo::EntityComponentManager &_ecm)
{
    // 坐标系说明：
    // - 世界坐标系（World Frame）：固定的全局坐标系，Z轴向上
    // - 本体坐标系（Robot Frame）：随机器人移动，X轴为前进方向，Y轴为左侧，Z轴向上
    // 
    // 控制策略：
    // 1. 在本体坐标系中计算控制力（例如：前进力在X轴，向上力在Z轴）
    // 2. 使用TransformForceToWorld将力转换到世界坐标系
    // 3. 在世界坐标系中施加力
    // 
    // 这样可以确保机器人无论朝向哪个方向，都能沿着自己的前进方向爬楼梯
    
    // 周期性发布当前状态（每0.5秒）
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatusPublishTime).count();
    if (elapsed > 500)  // 500ms
    {
        // 根据当前状态机状态确定要发布的状态
        ClimberStatus current_status = ClimberStatus::IDLE;
        if (state == SimpleClimbState::IDLE || state == SimpleClimbState::COMPLETE)
        {
            current_status = ClimberStatus::IDLE;
        }
        else
        {
            current_status = ClimberStatus::RUNNING;
        }
        PublishStatus(current_status, true);  // 强制发布
    }
    
    auto pose = _ecm.Component<components::WorldPose>(this->chassisLink)->Data();
    auto velocity = _ecm.Component<components::LinearVelocity>(this->chassisLink)->Data();
    auto angularVel = _ecm.Component<components::AngularVelocity>(this->chassisLink)->Data();
    
    double currentZ = pose.Z();
    double currentX = pose.X();
    double currentVelZ = velocity.Z();
    
    // 获取当前姿态角（roll, pitch, yaw）
    auto rotation = pose.Rot();
    
    // 直接使用Ignition的欧拉角
    double roll = rotation.Roll();
    double pitch = rotation.Pitch();
    double yaw = rotation.Yaw();
    
    // 关键：将世界坐标系的角速度转换到机器人本体坐标系
    // 这样PD控制器使用的是本体坐标系的角速度
    math::Vector3d robotAngularVel = rotation.RotateVectorReverse(angularVel);
    
    // 实时打印角度（每0.1秒）- 适用于所有状态
    static auto lastAnglePrintTime = std::chrono::steady_clock::now();
    auto angleNow = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(angleNow - lastAnglePrintTime).count() > 100)
    {
        lastAnglePrintTime = angleNow;
    }
    
    // 姿态稳定力矩（PD控制，保持水平）
    // 在机器人本体坐标系中计算力矩
    // 目标：roll = 0, pitch = 0
    math::Vector3d robotStabilizeTorque(
        -ANGLE_STABILIZE_KP * roll - ANGLE_STABILIZE_KD * robotAngularVel.X(),  // 抵消roll（绕机器人X轴）
        -ANGLE_STABILIZE_KP * pitch - ANGLE_STABILIZE_KD * robotAngularVel.Y(), // 抵消pitch（绕机器人Y轴）
        0.0  // 不控制yaw
    );
    
    // 限制力矩大小，防止力矩爆炸
    const double MAX_TORQUE = 100.0;  // 最大力矩 (N·m)
    double torqueMagnitude = robotStabilizeTorque.Length();
    if (torqueMagnitude > MAX_TORQUE) {
        robotStabilizeTorque = robotStabilizeTorque * (MAX_TORQUE / torqueMagnitude);
    }
    
    // 只在CLIMB_FORWARD和DESCEND_BACKWARD状态下施加姿态稳定力矩
    // CLIMB_UP和DESCEND_DOWN时不需要保持水平
    if (state == SimpleClimbState::CLIMB_FORWARD || state == SimpleClimbState::DESCEND_BACKWARD)
    {
        // 根据yaw角度范围使用不同的控制策略
        // yaw的范围是[-π, π]，即[-180°, 180°]
        
        // 将yaw归一化到[-π, π]范围
        double normalizedYaw = yaw;
        while (normalizedYaw > M_PI) normalizedYaw -= 2 * M_PI;
        while (normalizedYaw < -M_PI) normalizedYaw += 2 * M_PI;
        
        // 判断是否在"问题区域"（yaw在90°到270°之间，即与x负半轴夹角<90°）
        // 这对应于yaw在[π/2, π]或[-π, -π/2]范围内
        bool inProblemZone = (normalizedYaw > M_PI/2) || (normalizedYaw < -M_PI/2);
        
        if (inProblemZone)
        {
            // 在问题区域：使用极其保守的参数，或者完全禁用
            // 选项1：使用极低的增益
            const double ULTRA_CONSERVATIVE_KP = 5.0;   // 极低的比例增益
            const double ULTRA_CONSERVATIVE_KD = 1.0;   // 极低的微分增益
            const double ULTRA_MAX_TORQUE = 20.0;       // 极低的力矩限制
            
            // 使用重力向量方法计算倾斜
            math::Vector3d worldGravity(0, 0, -1);
            math::Vector3d robotGravity = rotation.RotateVectorReverse(worldGravity);
            
            double gravityZ = std::abs(robotGravity.Z());
            if (gravityZ < 0.1) gravityZ = 0.1;
            
            double tiltX = robotGravity.X() / gravityZ;
            double tiltY = robotGravity.Y() / gravityZ;
            
            math::Vector3d ultraConservativeTorque(
                -ULTRA_CONSERVATIVE_KP * tiltY - ULTRA_CONSERVATIVE_KD * robotAngularVel.X(),
                -ULTRA_CONSERVATIVE_KP * tiltX - ULTRA_CONSERVATIVE_KD * robotAngularVel.Y(),
                0.0
            );
            
            double torqueMagnitude = ultraConservativeTorque.Length();
            if (torqueMagnitude > ULTRA_MAX_TORQUE) {
                ultraConservativeTorque = ultraConservativeTorque * (ULTRA_MAX_TORQUE / torqueMagnitude);
            }
            
            // 只在非常小的倾斜时才施加力矩
            if (std::abs(tiltX) < 0.2 && std::abs(tiltY) < 0.2) {
                ApplyTorque(_ecm, ultraConservativeTorque);
            }
        }
        else
        {
            // 在正常区域：使用标准的保守参数
            const double CONSERVATIVE_KP = 20.0;
            const double CONSERVATIVE_KD = 5.0;
            
            // 使用重力向量方法计算倾斜
            math::Vector3d worldGravity(0, 0, -1);
            math::Vector3d robotGravity = rotation.RotateVectorReverse(worldGravity);
            
            double gravityZ = std::abs(robotGravity.Z());
            if (gravityZ < 0.1) gravityZ = 0.1;
            
            double tiltX = robotGravity.X() / gravityZ;
            double tiltY = robotGravity.Y() / gravityZ;
            
            math::Vector3d conservativeTorque(
                -CONSERVATIVE_KP * tiltY - CONSERVATIVE_KD * robotAngularVel.X(),
                -CONSERVATIVE_KP * tiltX - CONSERVATIVE_KD * robotAngularVel.Y(),
                0.0
            );
            
            double torqueMagnitude = conservativeTorque.Length();
            if (torqueMagnitude > MAX_TORQUE) {
                conservativeTorque = conservativeTorque * (MAX_TORQUE / torqueMagnitude);
            }
            
            if (std::abs(tiltX) < 0.5 && std::abs(tiltY) < 0.5) {
                ApplyTorque(_ecm, conservativeTorque);
            }
        }
    }
    
    switch (state)
    {
    case SimpleClimbState::IDLE:
        // 空闲状态，不施加任何力
        break;
        
    case SimpleClimbState::CLIMB_UP:
        {
            // 上楼梯：向上移动
            double deltaZ = currentZ - startZ;
            
            if (deltaZ < targetHeight - POSITION_TOLERANCE)
            {
                // 还没到达目标高度，继续向上施力
                // 使用带速度限制的力控制
                double forceZ = VERTICAL_FORCE;
                
                // 如果速度超过最大值，施加反向阻尼力
                if (currentVelZ > MAX_VERTICAL_VELOCITY)
                {
                    forceZ = -VELOCITY_DAMPING * (currentVelZ - MAX_VERTICAL_VELOCITY);
                }
                // 如果接近目标，减小力度
                else if (deltaZ > targetHeight - 0.1)
                {
                    forceZ = VERTICAL_FORCE * 0.3;
                }
                
                // 垂直方向的力不需要坐标转换，直接在世界坐标系中施加
                math::Vector3d worldForce(0, 0, 130);
                ApplyForce(_ecm, worldForce);
            }
            else
            {
                // 到达目标高度，施加阻尼力停止
                if (std::abs(currentVelZ) > 0.01)
                {
                    // ApplyForce(_ecm, math::Vector3d(0, 0, 0));
                }
                else
                {
                    // 速度足够小，但是需要检查姿态是否稳定
                    // 如果roll或pitch角度太大，继续等待
                    if (std::abs(roll) > 0.1 || std::abs(pitch) > 0.1)  // 0.1弧度 ≈ 5.7度
                    {
                        // 姿态不稳定，施加稳定力矩并保持高度
                        // 在机器人本体坐标系中计算稳定力矩
                        math::Vector3d robotStabilizeTorque(
                            -ANGLE_STABILIZE_KP * roll - ANGLE_STABILIZE_KD * robotAngularVel.X(),
                            -ANGLE_STABILIZE_KP * pitch - ANGLE_STABILIZE_KD * robotAngularVel.Y(),
                            0.0
                        );
                        
                        // 限制力矩大小
                        double torqueMagnitude = robotStabilizeTorque.Length();
                        if (torqueMagnitude > MAX_TORQUE) {
                            robotStabilizeTorque = robotStabilizeTorque * (MAX_TORQUE / torqueMagnitude);
                        }
                        
                        // ApplyTorque会自动转换到世界坐标系
                        ApplyTorque(_ecm, robotStabilizeTorque);
                        
                        // 保持高度
                        double heightError = (startZ + targetHeight) - currentZ;
                        double heightForce = GRAVITY_COMPENSATION + HEIGHT_HOLD_KP * heightError - HEIGHT_HOLD_KD * currentVelZ;
                        heightForce = std::clamp(heightForce, -MAX_HEIGHT_FORCE, MAX_HEIGHT_FORCE);
                        ApplyForce(_ecm, math::Vector3d(0, 0, heightForce));
                    }
                    else
                    {
                        // 姿态稳定，切换到前进阶段
                        state = SimpleClimbState::CLIMB_FORWARD;
                        startX = currentX;
                        startY = pose.Y();  // 保存当前Y坐标作为前进阶段的起点
                        startZ = currentZ;  // 保存当前高度作为前进阶段要保持的高度
                        startYaw = GetRobotYaw(pose);  // 保存当前yaw角度，用于整个前进阶段
                        ignmsg << "SimpleStairClimber: Climb up complete (height: " << deltaZ 
                               << "m), moving forward..." << std::endl;
                    }
                }
            }
        }
        break;
        
    case SimpleClimbState::CLIMB_FORWARD:
        {
            // 上楼梯：向前移动（同时主动保持高度）
            // 使用当前的yaw角度，确保力和速度在同一坐标系
            double yaw = GetRobotYaw(pose);
            
            // 计算世界坐标系位移（使用保存的startYaw来计算位移）
            double worldDeltaX = pose.X() - startX;
            double worldDeltaY = pose.Y() - startY;
            
            // 使用保存的startYaw计算位移（保证位移测量的一致性）
            double cos_start_yaw = std::cos(startYaw);
            double sin_start_yaw = std::sin(startYaw);
            double robotDeltaX = worldDeltaX * cos_start_yaw + worldDeltaY * sin_start_yaw;
            
            // 将世界坐标系速度转换到机器人坐标系
            double robotVelX = velocity.X();
            
            if (robotDeltaX < HORIZONTAL_DISTANCE - POSITION_TOLERANCE)
            {
                // 还没到达目标距离，继续向前施力
                double forceX = HORIZONTAL_FORCE;
                
                // 如果速度超过最大值，施加反向阻尼力
                if (robotVelX > MAX_HORIZONTAL_VELOCITY)
                {
                    forceX = -VELOCITY_DAMPING * (robotVelX - MAX_HORIZONTAL_VELOCITY);
                }
                // 如果接近目标，减小力度
                else if (robotDeltaX > HORIZONTAL_DISTANCE - 0.1)
                {
                    forceX = HORIZONTAL_FORCE * 0.3;
                }
                
                // 高度保持控制器（PD控制）
                double heightError = startZ - currentZ;  // 期望高度 - 当前高度
                double heightForce = GRAVITY_COMPENSATION + HEIGHT_HOLD_KP * heightError - HEIGHT_HOLD_KD * currentVelZ;
                // 限制高度保持力，防止力爆炸
                heightForce = std::clamp(heightForce, -MAX_HEIGHT_FORCE, MAX_HEIGHT_FORCE);
                
                // 本体坐标系：X方向的力（前进） + Z方向的高度保持力
                math::Vector3d robotForce(forceX, 0, heightForce);
                // 转换到世界坐标系
                math::Vector3d worldForce = TransformForceToWorld(robotForce, yaw);
                
                ApplyForce(_ecm, worldForce);
            }
            else
            {
                // 到达目标距离，施加阻尼力停止（同时保持高度）
                if (std::abs(robotVelX) > 0.01)
                {
                    double heightError = startZ - currentZ;
                    double heightForce = GRAVITY_COMPENSATION + HEIGHT_HOLD_KP * heightError - HEIGHT_HOLD_KD * currentVelZ;
                    // 限制高度保持力
                    heightForce = std::clamp(heightForce, -MAX_HEIGHT_FORCE, MAX_HEIGHT_FORCE);
                    
                    // 本体坐标系：阻尼力 + 高度保持力
                    math::Vector3d robotForce(-VELOCITY_DAMPING * robotVelX, 0, heightForce);
                    math::Vector3d worldForce = TransformForceToWorld(robotForce, yaw);
                    ApplyForce(_ecm, worldForce);
                }
                else
                {
                    // 速度足够小，完成
                    state = SimpleClimbState::COMPLETE;
                    ignmsg << "SimpleStairClimber: Climb forward complete (distance: " << robotDeltaX 
                           << "m), climb finished!" << std::endl;
                }
            }
        }
        break;
        
    case SimpleClimbState::DESCEND_BACKWARD:
        {
            // 下楼梯：向后移动（同时主动保持高度）
            // 使用保存的yaw角度，确保整个后退阶段使用一致的坐标系
            double yaw = startYaw;
            
            // 计算世界坐标系位移（后退方向）
            double worldDeltaX = startX - currentX;  // 注意：倒退时是 startX - currentX
            double worldDeltaY = startY - pose.Y();  // 使用保存的startY
            
            // 转换到本体坐标系（后退距离）
            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);
            double robotDeltaX = worldDeltaX * cos_yaw + worldDeltaY * sin_yaw;  // 本体X方向（后退距离）
            
            // 将世界坐标系速度转换到机器人坐标系
            // 后退时robotVelX应该是负数
            double robotVelX = velocity.X();
            
            if (robotDeltaX < HORIZONTAL_DISTANCE - POSITION_TOLERANCE)
            {
                // 还没到达目标距离，继续向后施力
                double forceX = -HORIZONTAL_FORCE;  // 负值表示后退
                
                // 如果速度超过最大值（负方向），施加反向阻尼力
                if (robotVelX < -MAX_HORIZONTAL_VELOCITY)
                {
                    forceX = -VELOCITY_DAMPING * (robotVelX + MAX_HORIZONTAL_VELOCITY);
                }
                // 如果接近目标，减小力度
                else if (robotDeltaX > HORIZONTAL_DISTANCE - 0.1)
                {
                    forceX = -HORIZONTAL_FORCE * 0.3;
                }
                
                // 高度保持控制器（PD控制）
                double heightError = startZ - currentZ;  // 期望高度 - 当前高度
                double heightForce = GRAVITY_COMPENSATION + HEIGHT_HOLD_KP * heightError - HEIGHT_HOLD_KD * currentVelZ;
                // 限制高度保持力
                heightForce = std::clamp(heightForce, -MAX_HEIGHT_FORCE, MAX_HEIGHT_FORCE);
                
                // 本体坐标系：-X方向的力（后退） + Z方向的高度保持力
                math::Vector3d robotForce(forceX, 0, heightForce);
                // 转换到世界坐标系
                math::Vector3d worldForce = TransformForceToWorld(robotForce, yaw);
                ApplyForce(_ecm, worldForce);
            }
            else
            {
                // 到达目标距离，施加阻尼力停止（同时保持高度）
                if (std::abs(robotVelX) > 0.01)
                {
                    double heightError = startZ - currentZ;
                    double heightForce = GRAVITY_COMPENSATION + HEIGHT_HOLD_KP * heightError - HEIGHT_HOLD_KD * currentVelZ;
                    // 限制高度保持力
                    heightForce = std::clamp(heightForce, -MAX_HEIGHT_FORCE, MAX_HEIGHT_FORCE);
                    
                    // 本体坐标系：阻尼力 + 高度保持力
                    math::Vector3d robotForce(-VELOCITY_DAMPING * robotVelX, 0, heightForce);
                    math::Vector3d worldForce = TransformForceToWorld(robotForce, yaw);
                    ApplyForce(_ecm, worldForce);
                }
                else
                {
                    // 速度足够小，切换到下降阶段
                    state = SimpleClimbState::DESCEND_DOWN;
                    startZ = currentZ;
                    ignmsg << "SimpleStairClimber: Descend backward complete (distance: " << robotDeltaX 
                           << "m), moving down..." << std::endl;
                }
            }
        }
        break;
        
    case SimpleClimbState::DESCEND_DOWN:
        {
            // 下楼梯：向下移动
            double deltaZ = startZ - currentZ;  // 注意：下降时是 startZ - currentZ
            
            if (deltaZ < targetHeight - POSITION_TOLERANCE)
            {
                // 还没到达目标高度，继续向下施力
                double forceZ = -VERTICAL_FORCE;
                
                // 如果速度超过最大值（负方向），施加反向阻尼力
                if (currentVelZ < -MAX_VERTICAL_VELOCITY)
                {
                    forceZ = -VELOCITY_DAMPING * (currentVelZ + MAX_VERTICAL_VELOCITY);
                }
                // 如果接近目标，减小力度
                else if (deltaZ > targetHeight - 0.1)
                {
                    forceZ = -VERTICAL_FORCE * 0.3;
                }
                
                // 垂直方向的力不需要坐标转换，直接在世界坐标系中施加
                math::Vector3d worldForce(0, 0, forceZ);
                ApplyForce(_ecm, worldForce);
            }
            else
            {
                // 到达目标高度，施加阻尼力停止
                if (std::abs(currentVelZ) > 0.01)
                {
                    ApplyForce(_ecm, math::Vector3d(0, 0, -VELOCITY_DAMPING * currentVelZ));
                }
                else
                {
                    // 速度足够小，完成
                    state = SimpleClimbState::COMPLETE;
                    ignmsg << "SimpleStairClimber: Descend down complete (height: " << deltaZ 
                           << "m), descend finished!" << std::endl;
                }
            }
        }
        break;
        
    case SimpleClimbState::COMPLETE:
        // 完成状态，发布成功状态
        PublishStatus(ClimberStatus::SUCCESS);
        ignmsg << "SimpleStairClimber: ===== OPERATION COMPLETE! =====" << std::endl;
        
        // 保持 SUCCESS 状态一段时间，然后重置为 IDLE
        state = SimpleClimbState::IDLE;
        PublishStatus(ClimberStatus::IDLE);
        ignmsg << "SimpleStairClimber: Reset to IDLE" << std::endl;
        break;
    }
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(SimpleStairClimber,
                    ignition::gazebo::System,
                    SimpleStairClimber::ISystemConfigure,
                    SimpleStairClimber::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SimpleStairClimber, "ignition::gazebo::systems::SimpleStairClimber")
