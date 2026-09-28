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

#ifndef IGNITION_GAZEBO_SYSTEMS_SIMPLESTAIRCLIMBER_HH_
#define IGNITION_GAZEBO_SYSTEMS_SIMPLESTAIRCLIMBER_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
    // Forward declaration
    class SimpleStairClimberPrivate;

    /// \brief 简化的爬楼梯插件
    /// 
    /// 通过直接控制机器人整体位置来实现爬楼梯和下楼梯
    /// 
    /// ## 系统参数
    /// 
    /// - <chassis_link>: 底盘 link 名称（必需）
    /// 
    /// ## 订阅话题
    /// 
    /// - /{model_name}/climb_stair (ignition.msgs.Double): 上楼梯指令，参数为台阶高度
    /// - /{model_name}/descend_stair (ignition.msgs.Double): 下楼梯指令，参数为台阶高度
    /// 
    /// ## 运动逻辑
    /// 
    /// 上楼梯：
    /// 1. 向上移动 stairHeight 距离
    /// 2. 向前移动 0.5m 距离
    /// 
    /// 下楼梯：
    /// 1. 向后移动 0.5m 距离
    /// 2. 向下移动 stairHeight 距离
    class SimpleStairClimber
        : public System,
          public ISystemConfigure,
          public ISystemPreUpdate
    {
        public: SimpleStairClimber();

        public: ~SimpleStairClimber() override = default;

        public: void Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &_eventMgr) override;

        public: void PreUpdate(
                    const ignition::gazebo::UpdateInfo &_info,
                    ignition::gazebo::EntityComponentManager &_ecm) override;

        private: std::unique_ptr<SimpleStairClimberPrivate> dataPtr;
    };
}
}
}
}

#endif
