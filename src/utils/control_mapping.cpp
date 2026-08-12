#include <libcamera_ros_driver/utils/control_mapping.h>

#include <array>
#include <format>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

using namespace std::string_view_literals; // Enables the "sv" suffix for std::string_view

template <typename Entries>
auto lookupEnum(std::string_view control_value, const Entries& mapping, std::string_view parameter)
{
  for (const auto& [key, enum_val] : mapping)
  {
    if (key == control_value)
      return enum_val;
  }

  auto msg = std::format("Cannot map value '{}' to enum for parameter '{}'", control_value, parameter);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("control_mapping"), msg);
  throw std::runtime_error(msg);
}

libcamera::controls::AeExposureModeEnum get_ae_exposure_mode(const std::string_view control_value)
{
  constexpr std::array map = {
      std::pair{"normal"sv, libcamera::controls::ExposureNormal},
      std::pair{"short"sv, libcamera::controls::ExposureShort},
      std::pair{"long"sv, libcamera::controls::ExposureLong},
      std::pair{"custom"sv, libcamera::controls::ExposureCustom},
  };

  return lookupEnum(control_value, map, "AeExposureMode");
}

libcamera::controls::AeMeteringModeEnum get_ae_metering_mode(const std::string_view control_value)
{
  constexpr std::array map = {
      std::pair{"centre-weighted"sv, libcamera::controls::MeteringCentreWeighted},
      std::pair{"spot"sv, libcamera::controls::MeteringSpot},
      std::pair{"matrix"sv, libcamera::controls::MeteringMatrix},
      std::pair{"custom"sv, libcamera::controls::MeteringCustom},
  };

  return lookupEnum(control_value, map, "AeMeteringMode");
}

libcamera::controls::AeConstraintModeEnum get_ae_constraint_mode(const std::string_view control_value)
{
  constexpr std::array map = {
      std::pair{"normal"sv, libcamera::controls::ConstraintNormal},
      std::pair{"highlight"sv, libcamera::controls::ConstraintHighlight},
      std::pair{"shadows"sv, libcamera::controls::ConstraintShadows},
      std::pair{"custom"sv, libcamera::controls::ConstraintCustom},
  };

  return lookupEnum(control_value, map, "AeConstraintMode");
}

libcamera::controls::AwbModeEnum get_awb_mode(const std::string_view control_value)
{
  constexpr std::array map = {
      std::pair{"auto"sv, libcamera::controls::AwbAuto},         std::pair{"incandescent"sv, libcamera::controls::AwbIncandescent},
      std::pair{"tungsten"sv, libcamera::controls::AwbTungsten}, std::pair{"fluorescent"sv, libcamera::controls::AwbFluorescent},
      std::pair{"indoor"sv, libcamera::controls::AwbIndoor},     std::pair{"daylight"sv, libcamera::controls::AwbDaylight},
      std::pair{"cloudy"sv, libcamera::controls::AwbCloudy},     std::pair{"custom"sv, libcamera::controls::AwbCustom},
  };

  return lookupEnum(control_value, map, "AwbMode");
}