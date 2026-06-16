#include <libcamera_ros_driver/utils/control_mapping.h>
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

libcamera::controls::AeExposureModeEnum get_ae_exposure_mode(const std::string& mode)
{
  static const std::unordered_map<std::string, libcamera::controls::AeExposureModeEnum> mode_map = {
      {"normal", (libcamera::controls::AeExposureModeEnum)0},
      {"short", (libcamera::controls::AeExposureModeEnum)1},
      {"long", (libcamera::controls::AeExposureModeEnum)2},
      {"custom", (libcamera::controls::AeExposureModeEnum)3},
  };

  try
  {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range&)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("control_mapping"), "invalid ae exposure mode: \"" << mode << "\"");
    throw std::runtime_error("invalid ae exposure mode: \"" + mode + "\"");
  }
}

libcamera::controls::AeMeteringModeEnum get_ae_metering_mode(const std::string& mode)
{
  static const std::unordered_map<std::string, libcamera::controls::AeMeteringModeEnum> mode_map = {
      {"centre-weighted", (libcamera::controls::AeMeteringModeEnum)0},
      {"spot", (libcamera::controls::AeMeteringModeEnum)1},
      {"matrix", (libcamera::controls::AeMeteringModeEnum)2},
      {"custom", (libcamera::controls::AeMeteringModeEnum)3},
  };

  try
  {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range&)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("control_mapping"), "invalid ae metering mode: \"" << mode << "\"");
    throw std::runtime_error("invalid ae metering mode: \"" + mode + "\"");
  }
}

libcamera::controls::AeConstraintModeEnum get_ae_constraint_mode(const std::string& mode)
{
  static const std::unordered_map<std::string, libcamera::controls::AeConstraintModeEnum> mode_map = {
      {"normal", (libcamera::controls::AeConstraintModeEnum)0},
      {"highlight", (libcamera::controls::AeConstraintModeEnum)1},
      {"shadows", (libcamera::controls::AeConstraintModeEnum)2},
      {"custom", (libcamera::controls::AeConstraintModeEnum)3},
  };

  try
  {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range&)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("control_mapping"), "invalid ae constraint mode: \"" << mode << "\"");
    throw std::runtime_error("invalid ae constraint mode: \"" + mode + "\"");
  }
}

libcamera::controls::AwbModeEnum get_awb_mode(const std::string& mode)
{
  static const std::unordered_map<std::string, libcamera::controls::AwbModeEnum> mode_map = {
      {"auto", (libcamera::controls::AwbModeEnum)0},         {"incandescent", (libcamera::controls::AwbModeEnum)1},
      {"tungsten", (libcamera::controls::AwbModeEnum)2}, {"fluorescent", (libcamera::controls::AwbModeEnum)3},
      {"indoor", (libcamera::controls::AwbModeEnum)4},     {"daylight", (libcamera::controls::AwbModeEnum)5},
      {"cloudy", (libcamera::controls::AwbModeEnum)6},     {"custom", (libcamera::controls::AwbModeEnum)7},
  };

  try
  {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range&)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("control_mapping"), "invalid awb mode: \"" << mode << "\"");
    throw std::runtime_error("invalid awb mode: \"" + mode + "\"");
  }
}
