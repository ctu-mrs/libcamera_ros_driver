#pragma once

#include <libcamera/control_ids.h>

template <typename Entries>
auto lookupEnum(std::string_view control_value, const Entries& mapping, std::string_view parameter);

libcamera::controls::AeExposureModeEnum get_ae_exposure_mode(const std::string_view control_value);

libcamera::controls::AeMeteringModeEnum get_ae_metering_mode(const std::string_view control_value);

libcamera::controls::AeConstraintModeEnum get_ae_constraint_mode(const std::string_view control_value);

libcamera::controls::AwbModeEnum get_awb_mode(const std::string_view control_value);
