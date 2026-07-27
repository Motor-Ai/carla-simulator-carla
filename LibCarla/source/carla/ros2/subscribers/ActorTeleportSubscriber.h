// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <vector>

#include "carla/ros2/subscribers/SubscriberBase.h"
#include "carla/ros2/types/ActorDefinition.h"
#include "geometry_msgs/msg/PosePubSubTypes.h"

namespace carla {
namespace ros2 {

using ActorTeleportSubscriberImpl =
    DdsSubscriberImpl<geometry_msgs::msg::Pose, geometry_msgs::msg::PosePubSubType>;

class ActorTeleportSubscriber : public SubscriberBase<geometry_msgs::msg::Pose> {
public:
  explicit ActorTeleportSubscriber(ROS2NameRecord& parent,
                                    carla::ros2::types::ActorTeleportCallback actor_teleport_callback);
  virtual ~ActorTeleportSubscriber() = default;

  /**
   * Implements SubscriberBase::ProcessMessages()
   */
  void ProcessMessages() override;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

private:
  std::shared_ptr<ActorTeleportSubscriberImpl> _impl;
  carla::ros2::types::ActorTeleportCallback _actor_teleport_callback;
};
}  // namespace ros2
}  // namespace carla
