// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>

#include "carla/ros2/services/ServiceBase.h"
#include "carla_msgs/srv/EnvironmentObjectPubSubTypes.h"

namespace carla {
namespace ros2 {

using EnvironmentObjectServiceImpl =
    DdsServiceImpl<carla_msgs::srv::EnvironmentObject_Request, carla_msgs::srv::EnvironmentObject_RequestPubSubType,
                   carla_msgs::srv::EnvironmentObject_Response, carla_msgs::srv::EnvironmentObject_ResponsePubSubType>;

class EnvironmentObjectService
  : public ServiceBase<carla_msgs::srv::EnvironmentObject_Request, carla_msgs::srv::EnvironmentObject_Response> {
public:
  EnvironmentObjectService(carla::rpc::RpcServerInterface &carla_server,
                           std::shared_ptr<carla::ros2::types::ActorNameDefinition> actor_name_definition);
  virtual ~EnvironmentObjectService() = default;

  /**
   * Implements ServiceInterface::CheckRequest() interface
   */
  void CheckRequest() override;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

private:
  carla_msgs::srv::EnvironmentObject_Response EnableEnvironmentObjects(
      carla_msgs::srv::EnvironmentObject_Request const &request);

  std::shared_ptr<EnvironmentObjectServiceImpl> _impl;
};

}  // namespace ros2
}  // namespace carla
