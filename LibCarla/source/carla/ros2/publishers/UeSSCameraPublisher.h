// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/publishers/UePublisherBaseCamera.h"
#include MSG_EXT(sensor_msgs/msg/CameraInfoPubSubTypes)
#include MSG_EXT(sensor_msgs/msg/ImagePubSubTypes)

namespace carla {
namespace ros2 {

class UeSSCameraPublisher : public UePublisherBaseCamera<sensor_msgs::msg::ImageFromBuffer::allocator_type> {
public:
  UeSSCameraPublisher(std::shared_ptr<carla::ros2::types::SensorActorDefinition> sensor_actor_definition,
                      std::shared_ptr<TransformPublisher> transform_publisher);
  virtual ~UeSSCameraPublisher() = default;
};
}  // namespace ros2
}  // namespace carla
