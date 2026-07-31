// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/subscribers/ActorTeleportSubscriber.h"

#include <algorithm>

#include "carla/ros2/impl/DdsSubscriberImpl.h"

namespace carla {
namespace ros2 {

int ActorTeleportSubscriber::s_in_flight_count = 0;
std::vector<ActorTeleportSubscriber*> ActorTeleportSubscriber::s_waiting_instances;

ActorTeleportSubscriber::ActorTeleportSubscriber(ROS2NameRecord& parent,
                                                   carla::ros2::types::ActorTeleportCallback actor_teleport_callback,
                                                   carla::ros2::types::ActorSetSimulatePhysicsCallback actor_set_simulate_physics_callback)
  : SubscriberBase(parent),
    _impl(std::make_shared<ActorTeleportSubscriberImpl>(*this)),
    _actor_teleport_callback(actor_teleport_callback),
    _actor_set_simulate_physics_callback(actor_set_simulate_physics_callback) {}

ActorTeleportSubscriber::~ActorTeleportSubscriber() {
  if (!_pending_reenable_physics) {
    return;
  }
  if (_waiting_at_barrier) {
    // already past my own one-tick hold, just waiting for everyone else - withdraw so the
    // barrier opening later can't call into freed memory.
    s_waiting_instances.erase(
        std::remove(s_waiting_instances.begin(), s_waiting_instances.end(), this), s_waiting_instances.end());
  }
  else {
    // still within my own first post-teleport tick, still counted against the barrier - remove
    // myself so a despawned actor can't permanently block every other vehicle's re-enable.
    --s_in_flight_count;
    MaybeOpenBarrier();
  }
}

bool ActorTeleportSubscriber::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  // reliable, not the default best-effort: a teleport is a one-shot critical command (like
  // CarlaControlSubscriber/CarlaSynchronizationWindowSubscriber, which override this for the same
  // reason) - a single dropped message under load means the leg never starts correctly at all,
  // unlike a lost frame of a high-rate sensor stream.
  return _impl->Init(domain_participant, get_topic_name("actor_teleport"), get_topic_qos().reliable());
}

void ActorTeleportSubscriber::FireDeferredReenablePhysics() {
  if (!_waiting_at_barrier) {
    // a newer teleport superseded this one before the barrier opened for it - nothing to do.
    return;
  }
  _waiting_at_barrier = false;
  _pending_reenable_physics = false;
  if (_actor_set_simulate_physics_callback != nullptr) {
    _actor_set_simulate_physics_callback(true);
  }
  else {
    carla::log_error("ActorTeleportSubscriber::FireDeferredReenablePhysics >> set_simulate_physics callback is not available!");
  }
}

void ActorTeleportSubscriber::MaybeOpenBarrier() {
  if ((s_in_flight_count == 0) && !s_waiting_instances.empty()) {
    auto waiting = std::move(s_waiting_instances);
    s_waiting_instances.clear();
    for (auto *instance : waiting) {
      instance->FireDeferredReenablePhysics();
    }
  }
}

void ActorTeleportSubscriber::ProcessMessages() {
  // this instance has held physics disabled for at least one tick now (see the header's doc
  // comment for why re-enabling can't happen in the same tick as the transform) - join the
  // barrier instead of re-enabling immediately, so this vehicle's physics can't wake up while
  // another vehicle teleported the same tick is still within its own first tick.
  if (_pending_reenable_physics && !_waiting_at_barrier) {
    _waiting_at_barrier = true;
    --s_in_flight_count;
    s_waiting_instances.push_back(this);
    MaybeOpenBarrier();
  }

  while (_impl->HasPublishersConnected() && _impl->HasNewMessage()) {
    if (_actor_teleport_callback != nullptr ) {
      carla::ros2::types::Transform transform(_impl->GetMessage());
      _actor_teleport_callback(transform);
      if (_waiting_at_barrier) {
        // this teleport supersedes the previous one before the barrier had a chance to open for
        // it - withdraw and restart this instance's own one-tick hold from scratch.
        s_waiting_instances.erase(
            std::remove(s_waiting_instances.begin(), s_waiting_instances.end(), this), s_waiting_instances.end());
        _waiting_at_barrier = false;
        ++s_in_flight_count;
      }
      else if (!_pending_reenable_physics) {
        ++s_in_flight_count;
      }
      _pending_reenable_physics = true;
    }
    else {
      carla::log_error("ActorTeleportSubscriber::ProcessMessages >> actor_teleport callback is not available!");
    }
  }
}

}  // namespace ros2
}  // namespace carla
