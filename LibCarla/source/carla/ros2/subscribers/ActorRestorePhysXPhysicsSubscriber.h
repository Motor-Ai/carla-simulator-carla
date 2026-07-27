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

using ActorRestorePhysXPhysicsSubscriberImpl =
    DdsSubscriberImpl<std_msgs::msg::Bool, std_msgs::msg::BoolPubSubType>;

class ActorRestorePhysXPhysicsSubscriber : public SubscriberBase<std_msgs::msg::Bool> {
public:
  explicit ActorRestorePhysXPhysicsSubscriber(ROS2NameRecord& parent,
                                    carla::ros2::types::ActorRestorePhysXPhysicsCallback actor_restore_physx_physics_callback);
  virtual ~ActorRestorePhysXPhysicsSubscriber() = default;

  /**
   * Implements SubscriberBase::ProcessMessages()
   */
  void ProcessMessages() override;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

private:
  std::shared_ptr<ActorRestorePhysXPhysicsSubscriberImpl> _impl;
  carla::ros2::types::ActorRestorePhysXPhysicsCallback _actor_restore_physx_physics_callback;
};
}  // namespace ros2
}  // namespace carla
