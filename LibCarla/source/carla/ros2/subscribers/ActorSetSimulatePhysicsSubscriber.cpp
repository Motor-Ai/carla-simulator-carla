// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/subscribers/ActorSetSimulatePhysicsSubscriber.h"

#include "carla/ros2/impl/DdsSubscriberImpl.h"

namespace carla {
namespace ros2 {

ActorSetSimulatePhysicsSubscriber::ActorSetSimulatePhysicsSubscriber(ROS2NameRecord& parent,
                                                   carla::ros2::types::ActorSetSimulatePhysicsCallback actor_set_simulate_physics_callback)
  : SubscriberBase(parent),
    _impl(std::make_shared<ActorSetSimulatePhysicsSubscriberImpl>(*this)),
    _actor_set_simulate_physics_callback(actor_set_simulate_physics_callback) {}

bool ActorSetSimulatePhysicsSubscriber::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  return _impl->Init(domain_participant, get_topic_name("set_simulate_physics"), get_topic_qos());
}

void ActorSetSimulatePhysicsSubscriber::ProcessMessages() {
  while (_impl->HasPublishersConnected() && _impl->HasNewMessage()) {
    if (_actor_set_simulate_physics_callback != nullptr ) {
      _actor_set_simulate_physics_callback(_impl->GetMessage().data());
    }
    else {
      carla::log_error("ActorSetSimulatePhysicsSubscriber::ProcessMessages >> set_simulate_physics callback is not available!");
    }
  }
}

}  // namespace ros2
}  // namespace carla
