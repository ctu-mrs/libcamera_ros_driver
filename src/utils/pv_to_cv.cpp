#include <libcamera_ros_driver/utils/pv_to_cv.h>
#include <libcamera_ros_driver/utils/types.h>
#include <cstdint>
#include <libcamera/base/span.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <ros/ros.h>
#include <vector>


libcamera::ControlValue pv_to_cv_int_array(const std::vector<int64_t> &values, const libcamera::ControlType &type) {
  // convert to Span (Integer32, Integer64) or geometric type Rectangle, Size
  switch (type) {
    case libcamera::ControlTypeInteger32:
      return {
        libcamera::Span<const CTInteger32>(std::vector<CTInteger32>(values.begin(), values.end()))};
    case libcamera::ControlTypeInteger64:
      return {libcamera::Span<const CTInteger64>(values)};
    case libcamera::ControlTypeRectangle:
      return {libcamera::Rectangle(values[0], values[1], values[2], values[3])};
    case libcamera::ControlTypeSize:
      return {libcamera::Size(values[0], values[1])};
    default:
      return {};
  }
}

libcamera::ControlValue pv_to_cv(const bool &parameter, const libcamera::ControlType &type){
  return {parameter};
}

libcamera::ControlValue pv_to_cv(const int &parameter, const libcamera::ControlType &type){
  if (type == libcamera::ControlTypeInteger32)
    return {CTInteger32(parameter)};
  else if (type == libcamera::ControlTypeInteger64)
    return {CTInteger64(parameter)};
  else
    return {};
}

libcamera::ControlValue pv_to_cv(const double &parameter, const libcamera::ControlType &type){
  return {CTFloat(parameter)};
}

libcamera::ControlValue pv_to_cv(const std::string &parameter, const libcamera::ControlType &type){
  return {parameter};
}

libcamera::ControlValue pv_to_cv(const std::vector<int64_t> &parameter, const libcamera::ControlType &type){
  return pv_to_cv_int_array(parameter, type);
}

libcamera::ControlValue pv_to_cv(const std::vector<double> &parameter, const libcamera::ControlType &type){
  return {libcamera::Span<const CTFloat>(std::vector<CTFloat>(
        parameter.begin(), parameter.end()))};
}

libcamera::ControlValue pv_to_cv(const std::vector<std::string> &parameter, const libcamera::ControlType &type){
  return {libcamera::Span<const CTString>(parameter)};
}
