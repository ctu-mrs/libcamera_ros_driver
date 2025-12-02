#include <libcamera_ros_driver/utils/stream_mapping.h>
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

libcamera::StreamRole get_role(const std::string &role){
  static const std::unordered_map<std::string, libcamera::StreamRole> roles_map = {
    {"raw", libcamera::StreamRole::Raw},
    {"still", libcamera::StreamRole::StillCapture},
    {"video", libcamera::StreamRole::VideoRecording},
    {"viewfinder", libcamera::StreamRole::Viewfinder},
  };

  try {
    return roles_map.at(role);
  }
  catch (const std::out_of_range &) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("stream_mapping"), "invalid stream role: \"" << role << "\"");
    throw std::runtime_error("invalid stream role: \"" + role + "\"");
  }
}

