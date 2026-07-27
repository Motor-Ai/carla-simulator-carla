// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/subscribers/ActorSetTargetAngularVelocitySubscriber.h"

#include "carla/ros2/impl/DdsSubscriberImpl.h"
#include "carla/geom/Math.h"

namespace carla {
namespace ros2 {

ActorSetTargetAngularVelocitySubscriber::ActorSetTargetAngularVelocitySubscriber(ROS2NameRecord& parent,
                                                   carla::ros2::types::ActorSetTargetAngularVelocityCallback actor_set_target_angular_velocity_callback)
  : SubscriberBase(parent),
    _impl(std::make_shared<ActorSetTargetAngularVelocitySubscriberImpl>(*this)),
    _actor_set_target_angular_velocity_callback(actor_set_target_angular_velocity_callback) {}

bool ActorSetTargetAngularVelocitySubscriber::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  return _impl->Init(domain_participant, get_topic_name("set_target_angular_velocity"), get_topic_qos());
}

void ActorSetTargetAngularVelocitySubscriber::ProcessMessages() {
  while (_impl->HasPublishersConnected() && _impl->HasNewMessage()) {
    if (_actor_set_target_angular_velocity_callback != nullptr ) {
      auto const &msg = _impl->GetMessage();
      // inverse of carla::ros2::types::AngularVelocity::angular_velocity()'s CARLA(rad)->ROS
      // mapping (x and z negated, y unchanged) - that mapping is its own inverse - followed by
      // the ROS rad/s -> CARLA deg/s unit conversion (see carla::geom::AngularVelocity's own doc).
      carla::geom::AngularVelocity const carla_angular_velocity(
          carla::geom::Math::ToDegrees(static_cast<float>(-msg.x())),
          carla::geom::Math::ToDegrees(static_cast<float>(msg.y())),
          carla::geom::Math::ToDegrees(static_cast<float>(-msg.z())));
      _actor_set_target_angular_velocity_callback(carla_angular_velocity);
    }
    else {
      carla::log_error("ActorSetTargetAngularVelocitySubscriber::ProcessMessages >> set_target_angular_velocity callback is not available!");
    }
  }
}

}  // namespace ros2
}  // namespace carla
