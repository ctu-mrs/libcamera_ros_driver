#pragma once

#include <string>
#include <libcamera/stream.h>

libcamera::StreamRole get_role(const std::string& role);
