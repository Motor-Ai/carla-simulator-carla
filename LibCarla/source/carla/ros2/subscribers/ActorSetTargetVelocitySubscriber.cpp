// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/subscribers/ActorSetTargetVelocitySubscriber.h"

#include "carla/ros2/impl/DdsSubscriberImpl.h"
#include "carla/ros2/types/CoordinateSystemTransform.h"

namespace carla {
namespace ros2 {

ActorSetTargetVelocitySubscriber::ActorSetTargetVelocitySubscriber(ROS2NameRecord& parent,
                                                   carla::ros2::types::ActorSetTargetVelocityCallback actor_set_target_velocity_callback)
  : SubscriberBase(parent),
    _impl(std::make_shared<ActorSetTargetVelocitySubscriberImpl>(*this)),
    _actor_set_target_velocity_callback(actor_set_target_velocity_callback) {}

bool ActorSetTargetVelocitySubscriber::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  // reliable, not the default best-effort: a one-shot critical actor command (like
  // ActorTeleportSubscriber, which overrides this for the same reason) - a single dropped message
  // under load leaves velocity in the wrong state with no follow-up message to correct it, unlike a
  // lost frame of a continuously-republished control stream.
  return _impl->Init(domain_participant, get_topic_name("set_target_velocity"), get_topic_qos().reliable());
}

void ActorSetTargetVelocitySubscriber::ProcessMessages() {
  while (_impl->HasPublishersConnected() && _impl->HasNewMessage()) {
    if (_actor_set_target_velocity_callback != nullptr ) {
      auto const &msg = _impl->GetMessage();
      carla::geom::Vector3D const ros_velocity(
          static_cast<float>(msg.x()), static_cast<float>(msg.y()), static_cast<float>(msg.z()));
      // ROS (right-handed, m/s) -> CARLA/UE4 (left-handed, m/s): the y-flip is its own inverse,
      // see CoordinateSystemTransform::TransformLinearAxixVector3D()'s CARLA->ROS direction.
      carla::geom::Vector3D const carla_velocity
          = carla::ros2::types::CoordinateSystemTransform::TransformLinearAxixVector3D(ros_velocity);
      _actor_set_target_velocity_callback(carla_velocity);
    }
    else {
      carla::log_error("ActorSetTargetVelocitySubscriber::ProcessMessages >> set_target_velocity callback is not available!");
    }
  }
}

}  // namespace ros2
}  // namespace carla
