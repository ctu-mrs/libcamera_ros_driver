#pragma once

#include <string>
#include <libcamera/control_ids.h>

libcamera::controls::AeExposureModeEnum get_ae_exposure_mode(const std::string &mode);

libcamera::controls::AeMeteringModeEnum get_ae_metering_mode(const std::string &mode);

libcamera::controls::AeConstraintModeEnum get_ae_constraint_mode(const std::string &mode);

libcamera::controls::AwbModeEnum get_awb_mode(const std::string &mode);

libcamera::controls::AfModeEnum get_af_mode(const std::string &mode);

libcamera::controls::AfRangeEnum get_af_range(const std::string &mode);

libcamera::controls::AfSpeedEnum get_af_speed(const std::string &mode);
