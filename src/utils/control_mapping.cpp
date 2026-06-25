#include <libcamera_ros_driver/utils/control_mapping.h>
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <ros/ros.h>

libcamera::controls::AeExposureModeEnum get_ae_exposure_mode(const std::string &mode){
  static const std::unordered_map<std::string, libcamera::controls::AeExposureModeEnum> mode_map = {
    {"normal", libcamera::controls::AeExposureModeEnum::ExposureNormal},
    {"short", libcamera::controls::AeExposureModeEnum::ExposureShort},
    {"long", libcamera::controls::AeExposureModeEnum::ExposureLong},
    {"custom", libcamera::controls::AeExposureModeEnum::ExposureCustom},
  };

  try {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range &) {
    ROS_ERROR_STREAM("invalid ae exposure mode: \"" << mode << "\"");
    throw std::runtime_error("invalid ae exposure mode: \"" + mode + "\"");
  }
}

libcamera::controls::AeMeteringModeEnum get_ae_metering_mode(const std::string &mode){
  static const std::unordered_map<std::string, libcamera::controls::AeMeteringModeEnum> mode_map = {
    {"centre-weighted", libcamera::controls::AeMeteringModeEnum::MeteringCentreWeighted},
    {"spot", libcamera::controls::AeMeteringModeEnum::MeteringSpot},
    {"matrix", libcamera::controls::AeMeteringModeEnum::MeteringMatrix},
    {"custom", libcamera::controls::AeMeteringModeEnum::MeteringCustom},
  };

  try {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range &) {
    ROS_ERROR_STREAM("invalid ae metering mode: \"" << mode << "\"");
    throw std::runtime_error("invalid ae metering mode: \"" + mode + "\"");
  }
}

libcamera::controls::AeConstraintModeEnum get_ae_constraint_mode(const std::string &mode){
  static const std::unordered_map<std::string, libcamera::controls::AeConstraintModeEnum> mode_map = {
    {"normal", libcamera::controls::AeConstraintModeEnum::ConstraintNormal},
    {"highlight", libcamera::controls::AeConstraintModeEnum::ConstraintHighlight},
    {"shadows", libcamera::controls::AeConstraintModeEnum::ConstraintShadows},
    {"custom", libcamera::controls::AeConstraintModeEnum::ConstraintCustom},
  };

  try {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range &) {
    ROS_ERROR_STREAM("invalid ae constraint mode: \"" << mode << "\"");
    throw std::runtime_error("invalid ae constraint mode: \"" + mode + "\"");
  }
}

libcamera::controls::AwbModeEnum get_awb_mode(const std::string &mode){
  static const std::unordered_map<std::string, libcamera::controls::AwbModeEnum> mode_map = {
    {"auto", libcamera::controls::AwbModeEnum::AwbAuto},
    {"incandescent", libcamera::controls::AwbModeEnum::AwbIncandescent},
    {"tungsten", libcamera::controls::AwbModeEnum::AwbTungsten},
    {"fluorescent", libcamera::controls::AwbModeEnum::AwbFluorescent},
    {"indoor", libcamera::controls::AwbModeEnum::AwbIndoor},
    {"daylight", libcamera::controls::AwbModeEnum::AwbDaylight},
    {"cloudy", libcamera::controls::AwbModeEnum::AwbCloudy},
    {"custom", libcamera::controls::AwbModeEnum::AwbCustom},
  };

  try {
    return mode_map.at(mode);
  }
  catch (const std::out_of_range &) {
    ROS_ERROR_STREAM("invalid awb mode: \"" << mode << "\"");
    throw std::runtime_error("invalid awb mode: \"" + mode + "\"");
  }
}
