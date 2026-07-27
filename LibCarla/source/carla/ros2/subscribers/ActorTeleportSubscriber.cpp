// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/subscribers/ActorTeleportSubscriber.h"

#include "carla/ros2/impl/DdsSubscriberImpl.h"

namespace carla {
namespace ros2 {

ActorTeleportSubscriber::ActorTeleportSubscriber(ROS2NameRecord& parent,
                                                   carla::ros2::types::ActorTeleportCallback actor_teleport_callback)
  : SubscriberBase(parent),
    _impl(std::make_shared<ActorTeleportSubscriberImpl>(*this)),
    _actor_teleport_callback(actor_teleport_callback) {}

bool ActorTeleportSubscriber::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  return _impl->Init(domain_participant, get_topic_name("actor_teleport"), get_topic_qos());
}

void ActorTeleportSubscriber::ProcessMessages() {
  while (_impl->HasPublishersConnected() && _impl->HasNewMessage()) {
    if (_actor_teleport_callback != nullptr ) {
      auto const &msg = _impl->GetMessage();
      carla::log_warning("ActorTeleportSubscriber::ProcessMessages >> received pose x=", msg.position().x(),
                         "y=", msg.position().y(), "z=", msg.position().z());
      carla::ros2::types::Transform transform(msg);
      _actor_teleport_callback(transform);
    }
    else {
      carla::log_error("ActorTeleportSubscriber::ProcessMessages >> actor_teleport callback is not available!");
    }
  }
}

}  // namespace ros2
}  // namespace carla
