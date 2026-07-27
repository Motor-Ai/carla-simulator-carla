// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <vector>

#include "carla/ros2/subscribers/SubscriberBase.h"
#include "carla/ros2/types/ActorDefinition.h"
#include "std_msgs/msg/BoolPubSubTypes.h"

namespace carla {
namespace ros2 {

using ActorSetSimulatePhysicsSubscriberImpl =
    DdsSubscriberImpl<std_msgs::msg::Bool, std_msgs::msg::BoolPubSubType>;

class ActorSetSimulatePhysicsSubscriber : public SubscriberBase<std_msgs::msg::Bool> {
public:
  explicit ActorSetSimulatePhysicsSubscriber(ROS2NameRecord& parent,
                                    carla::ros2::types::ActorSetSimulatePhysicsCallback actor_set_simulate_physics_callback);
  virtual ~ActorSetSimulatePhysicsSubscriber() = default;

  /**
   * Implements SubscriberBase::ProcessMessages()
   */
  void ProcessMessages() override;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

private:
  std::shared_ptr<ActorSetSimulatePhysicsSubscriberImpl> _impl;
  carla::ros2::types::ActorSetSimulatePhysicsCallback _actor_set_simulate_physics_callback;
};
}  // namespace ros2
}  // namespace carla
