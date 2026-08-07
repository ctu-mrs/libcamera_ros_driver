#pragma once

#include <libcamera/controls.h>

libcamera::ControlValue pv_to_cv(const bool& parameter, const libcamera::ControlType& type);
libcamera::ControlValue pv_to_cv(const int& parameter, const libcamera::ControlType& type);
libcamera::ControlValue pv_to_cv(const double& parameter, const libcamera::ControlType& type);
libcamera::ControlValue pv_to_cv(const std::string& parameter, const libcamera::ControlType& type);
libcamera::ControlValue pv_to_cv(const std::vector<int64_t>& parameter, const libcamera::ControlType& type);
libcamera::ControlValue pv_to_cv(const std::vector<double>& parameter, const libcamera::ControlType& type);
libcamera::ControlValue pv_to_cv(const std::vector<std::string>& parameter, const libcamera::ControlType& type);
