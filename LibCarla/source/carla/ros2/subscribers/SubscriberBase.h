// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <vector>

#include "carla/ros2/ROS2NameRecord.h"
#include "carla/ros2/ROS2QoS.h"

namespace carla {
namespace ros2 {

/**
  A Subscriber base class.
  */
template <typename MESSAGE_TYPE, typename MESSAGE_PUB_TYPE>
class DdsSubscriberImpl;

/**
 * Non-template interface shared by every SubscriberBase<MESSAGE_TYPE> instantiation - neither
 * Init() nor ProcessMessages() actually depends on MESSAGE_TYPE at this level. Lets a caller that
 * owns a heterogeneous set of subscribers (e.g. one vehicle's several control/set_* subscribers,
 * each a different message type) keep them in a single homogeneous list and Init()/ProcessMessages()
 * all of them generically, instead of one manually-named member per concrete subscriber type -
 * the latter is easy to silently forget to wire into a manual per-member dispatch list (as
 * happened for several vehicle subscribers before this was introduced).
 */
class ISubscriber {
public:
  virtual ~ISubscriber() = default;
  virtual bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) = 0;
  virtual void ProcessMessages() = 0;
};

/**
 * Subscriber Base class
 */
template <typename MESSAGE_TYPE>
class SubscriberBase : public ISubscriber {
public:
  SubscriberBase(ROS2NameRecord &parent) : _parent(parent) {
    log_debug("SubscriberBase created for topic {}", parent.get_topic_name());
  }
  virtual ~SubscriberBase() {
    log_debug("SubscriberBase destroyed for topic {}", _parent.get_topic_name());
  };

  /**
   * Initialze the subscriber
   */
  virtual bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) = 0;

  /**
   * Process all available messages.
   */
  virtual void ProcessMessages() = 0;

  /**
   * A new publisher has connected to this subscriber.
   */
  virtual void PublisherConnected(std::string const &publisher_guid) {
    (void)publisher_guid;
  }

  /**
   * A publisher has disconnected from this subscriber.
   */
  virtual void PublisherDisconnected(std::string const &publisher_guid) {
    (void)publisher_guid;
  }

  /*
   * @brief Default get_topic_qos() for subscribers
   *
   * Be aware: The default selection for subscribers is NOT as done default in ROS2 (which aims compatibility to ROS1)!
   * Per default, we want to achieve the most compatible combination within ROS2 world in the sense,
   * that receiption is possible for all possible publisher configurations.
   * https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-compatibilities
   *
   * Reliability::BEST_EFFORT
   * Durability::VOLATILE
   * History::KEEP_LAST, depth: 10u
   */
  ROS2QoS get_topic_qos() const {
    return DEFAULT_SUBSCRIBER_QOS;
  };

  std::string get_topic_name(std::string postfix = "") const {
    return _parent.get_topic_name(postfix);
  }

  carla::streaming::detail::actor_id_type get_actor_id() const {
    return _parent.get_actor_id();
  }

protected:
  ROS2NameRecord &_parent;
};
}  // namespace ros2
}  // namespace carla
