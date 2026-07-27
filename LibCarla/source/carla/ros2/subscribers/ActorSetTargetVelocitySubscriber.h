// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <vector>

#include "carla/ros2/subscribers/SubscriberBase.h"
#include "carla/ros2/types/ActorDefinition.h"
#include "geometry_msgs/msg/Vector3PubSubTypes.h"

namespace carla {
namespace ros2 {

using ActorSetTargetVelocitySubscriberImpl =
    DdsSubscriberImpl<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3PubSubType>;

class ActorSetTargetVelocitySubscriber : public SubscriberBase<geometry_msgs::msg::Vector3> {
public:
  explicit ActorSetTargetVelocitySubscriber(ROS2NameRecord& parent,
                                    carla::ros2::types::ActorSetTargetVelocityCallback actor_set_target_velocity_callback);
  virtual ~ActorSetTargetVelocitySubscriber() = default;

  /**
   * Implements SubscriberBase::ProcessMessages()
   */
  void ProcessMessages() override;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

private:
  std::shared_ptr<ActorSetTargetVelocitySubscriberImpl> _impl;
  carla::ros2::types::ActorSetTargetVelocityCallback _actor_set_target_velocity_callback;
};
}  // namespace ros2
}  // namespace carla
