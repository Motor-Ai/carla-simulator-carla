// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/subscribers/ActorRestorePhysXPhysicsSubscriber.h"

#include "carla/ros2/impl/DdsSubscriberImpl.h"

namespace carla {
namespace ros2 {

ActorRestorePhysXPhysicsSubscriber::ActorRestorePhysXPhysicsSubscriber(ROS2NameRecord& parent,
                                                   carla::ros2::types::ActorRestorePhysXPhysicsCallback actor_restore_physx_physics_callback)
  : SubscriberBase(parent),
    _impl(std::make_shared<ActorRestorePhysXPhysicsSubscriberImpl>(*this)),
    _actor_restore_physx_physics_callback(actor_restore_physx_physics_callback) {}

bool ActorRestorePhysXPhysicsSubscriber::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  // reliable, not the default best-effort: a one-shot critical actor command (like
  // ActorTeleportSubscriber, which overrides this for the same reason) - a single dropped message
  // under load leaves PhysX state unrestored with no follow-up message to correct it, unlike a lost
  // frame of a continuously-republished control stream.
  return _impl->Init(domain_participant, get_topic_name("restore_physx_physics"), get_topic_qos().reliable());
}

void ActorRestorePhysXPhysicsSubscriber::ProcessMessages() {
  while (_impl->HasPublishersConnected() && _impl->HasNewMessage()) {
    // restore_physx_physics has no payload of its own (see FVehicleActor::RestorePhysXPhysics()) -
    // data()==true triggers it; false is a no-op, so an accidental/default-constructed message
    // can't trigger a reset.
    if (!_impl->GetMessage().data()) {
      continue;
    }
    if (_actor_restore_physx_physics_callback != nullptr ) {
      _actor_restore_physx_physics_callback();
    }
    else {
      carla::log_error("ActorRestorePhysXPhysicsSubscriber::ProcessMessages >> restore_physx_physics callback is not available!");
    }
  }
}

}  // namespace ros2
}  // namespace carla
