// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <string>

#if FASTDDS_VERSION_MAJOR >= 3
# include <fastdds/dds/core/detail/DDSReturnCode.hpp>
# define FastDdsReturnCodePrefix eprosima::fastdds::dds
using FastDdsReturnCodeType = eprosima::fastdds::dds::ReturnCode_t;
namespace FastRtpsNamespace = eprosima::fastdds;
#else
# include <fastrtps/types/TypesBase.h>
# define FastDdsReturnCodePrefix FastRtpsNamespace::types::ReturnCode_t::ReturnCodeValue
using FastDdsReturnCodeType = eprosima::fastrtps::types::ReturnCode_t;
namespace FastRtpsNamespace = eprosima::fastrtps;
#endif

namespace carla {
namespace ros2 {

inline std::string return_code_string(FastDdsReturnCodeType rcode) {

#if FASTDDS_VERSION_MAJOR >= 3
  switch (rcode)
#else
  switch (rcode())
#endif
  {
    case FastDdsReturnCodePrefix::RETCODE_OK:
      return "RETCODE_OK";
    case FastDdsReturnCodePrefix::RETCODE_ERROR:
      return "RETCODE_ERROR";
    case FastDdsReturnCodePrefix::RETCODE_UNSUPPORTED:
      return "RETCODE_UNSUPPORTED";
    case FastDdsReturnCodePrefix::RETCODE_BAD_PARAMETER:
      return "RETCODE_BAD_PARAMETER";
    case FastDdsReturnCodePrefix::RETCODE_PRECONDITION_NOT_MET:
      return "RETCODE_PRECONDITION_NOT_MET";
    case FastDdsReturnCodePrefix::RETCODE_OUT_OF_RESOURCES:
      return "RETCODE_OUT_OF_RESOURCES";
    case FastDdsReturnCodePrefix::RETCODE_NOT_ENABLED:
      return "RETCODE_NOT_ENABLED";
    case FastDdsReturnCodePrefix::RETCODE_IMMUTABLE_POLICY:
      return "RETCODE_IMMUTABLE_POLICY";
    case FastDdsReturnCodePrefix::RETCODE_INCONSISTENT_POLICY:
      return "RETCODE_INCONSISTENT_POLICY";
    case FastDdsReturnCodePrefix::RETCODE_ALREADY_DELETED:
      return "RETCODE_ALREADY_DELETED";
    case FastDdsReturnCodePrefix::RETCODE_TIMEOUT:
      return "RETCODE_TIMEOUT";
    case FastDdsReturnCodePrefix::RETCODE_NO_DATA:
      return "RETCODE_NO_DATA";
    case FastDdsReturnCodePrefix::RETCODE_ILLEGAL_OPERATION:
      return "RETCODE_ILLEGAL_OPERATION";
    case FastDdsReturnCodePrefix::RETCODE_NOT_ALLOWED_BY_SECURITY:
      return "RETCODE_NOT_ALLOWED_BY_SECURITY";
    default:
      return "UNKNOWN";
  }
}

} // namespace ros2
} // namespace carla