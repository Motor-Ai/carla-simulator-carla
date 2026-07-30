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

// Re-enabling physics has to wait at least one tick after the teleport transform is applied -
// doing it in the same call can corrupt the actor's internal physics state (the collision/
// broadphase system needs a tick to register the actor at its new position first). This
// subscriber hides that requirement from its caller: ProcessMessages() applies the teleport
// (physics stays disabled) and arms a pending flag, then re-enables physics itself once ready -
// so a caller still only ever has to publish one message to get a complete, correctly-sequenced
// teleport.
//
// The "ready" timing is coordinated *globally*, across every vehicle's ActorTeleportSubscriber,
// via a shared barrier (s_in_flight_count/s_waiting_instances): an instance only actually
// re-enables once every other instance currently mid-teleport has also held for at least one
// tick, so one vehicle's physics can never wake up while another vehicle teleported the same
// tick is still supposed to be frozen. A newer teleport landing on an instance that's already
// waiting at the barrier withdraws it and restarts its own one-tick hold. The destructor
// withdraws this instance from whichever part of the barrier bookkeeping it's currently in, so a
// despawned actor can never leave the barrier permanently blocked for everyone else.
class ActorTeleportSubscriber : public SubscriberBase<geometry_msgs::msg::Pose> {
public:
  explicit ActorTeleportSubscriber(ROS2NameRecord& parent,
                                    carla::ros2::types::ActorTeleportCallback actor_teleport_callback,
                                    carla::ros2::types::ActorSetSimulatePhysicsCallback actor_set_simulate_physics_callback);
  virtual ~ActorTeleportSubscriber();

  /**
   * Implements SubscriberBase::ProcessMessages()
   */
  void ProcessMessages() override;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

private:
  // Fires this instance's own re-enable, but only if it's still actually waiting at the barrier -
  // a newer teleport landing on it in the meantime already withdrew it, making this a no-op.
  // Called on every currently-waiting instance once the barrier opens (see MaybeOpenBarrier()).
  void FireDeferredReenablePhysics();

  // If the barrier is no longer blocked (s_in_flight_count == 0) and someone is actually waiting
  // on it, fires every currently-waiting instance's deferred re-enable and clears the wait list.
  static void MaybeOpenBarrier();

  std::shared_ptr<ActorTeleportSubscriberImpl> _impl;
  carla::ros2::types::ActorTeleportCallback _actor_teleport_callback;
  carla::ros2::types::ActorSetSimulatePhysicsCallback _actor_set_simulate_physics_callback;
  // true from the tick a teleport is applied until this instance's re-enable has actually fired.
  bool _pending_reenable_physics{false};
  // true once this instance has held for at least one tick and joined s_waiting_instances -
  // false while still within its own first post-teleport tick, or once it has actually fired.
  bool _waiting_at_barrier{false};

  // number of ActorTeleportSubscriber instances (across all vehicles) currently within their own
  // first post-teleport tick - i.e. not yet even eligible to join the barrier.
  static int s_in_flight_count;
  // every instance that has held for at least one tick and is waiting for s_in_flight_count to
  // reach 0 before it may actually re-enable physics.
  static std::vector<ActorTeleportSubscriber*> s_waiting_instances;
};
}  // namespace ros2
}  // namespace carla
