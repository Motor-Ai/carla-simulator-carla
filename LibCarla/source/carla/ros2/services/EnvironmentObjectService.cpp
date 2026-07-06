// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/services/EnvironmentObjectService.h"

#include "carla/ros2/impl/DdsServiceImpl.h"

namespace carla {
namespace ros2 {

EnvironmentObjectService::EnvironmentObjectService(
    carla::rpc::RpcServerInterface &carla_server,
    std::shared_ptr<carla::ros2::types::ActorNameDefinition> actor_name_definition)
  : ServiceBase(carla_server, actor_name_definition),
    _impl(std::make_shared<EnvironmentObjectServiceImpl>()) {}

bool EnvironmentObjectService::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  _impl->SetSyncServiceCallback(
      std::bind(&EnvironmentObjectService::EnableEnvironmentObjects, this, std::placeholders::_1));
  return _impl->Init(domain_participant, get_topic_name());
}

void EnvironmentObjectService::CheckRequest() {
  _impl->CheckRequest();
}

carla_msgs::srv::EnvironmentObject_Response EnvironmentObjectService::EnableEnvironmentObjects(
    carla_msgs::srv::EnvironmentObject_Request const &request) {

  carla_msgs::srv::EnvironmentObject_Response response;
  auto result = _carla_server.call_enable_environment_objects(request.ids(), request.enable());
  if (result.HasError()) {
    log_error("ROS2:EnableEnvironmentObjects(): failed: ", result.GetError().What());
    response.success(false);
  } else {
    log_info("ROS2:EnableEnvironmentObjects(): success");
    response.success(true);
  }

  return response;
}

}  // namespace ros2
}  // namespace carla
