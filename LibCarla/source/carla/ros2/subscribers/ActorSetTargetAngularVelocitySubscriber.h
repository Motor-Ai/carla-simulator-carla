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

using ActorSetTargetAngularVelocitySubscriberImpl =
    DdsSubscriberImpl<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3PubSubType>;

class ActorSetTargetAngularVelocitySubscriber : public SubscriberBase<geometry_msgs::msg::Vector3> {
public:
  explicit ActorSetTargetAngularVelocitySubscriber(ROS2NameRecord& parent,
                                    carla::ros2::types::ActorSetTargetAngularVelocityCallback actor_set_target_angular_velocity_callback);
  virtual ~ActorSetTargetAngularVelocitySubscriber() = default;

  /**
   * Implements SubscriberBase::ProcessMessages()
   */
  void ProcessMessages() override;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

private:
  std::shared_ptr<ActorSetTargetAngularVelocitySubscriberImpl> _impl;
  carla::ros2::types::ActorSetTargetAngularVelocityCallback _actor_set_target_angular_velocity_callback;
};
}  // namespace ros2
}  // namespace carla
