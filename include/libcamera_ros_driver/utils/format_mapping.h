#pragma once

#include <libcamera/stream.h>
#include <string>

namespace libcamera
{
  class PixelFormat;
}

enum class FormatType
{
  NONE,
  RAW,
  COMPRESSED,
};

std::string get_ros_encoding(const libcamera::PixelFormat& pixelformat);

FormatType format_type(const libcamera::PixelFormat& pixelformat);

// Get the 16-bit format version of 8-bit format (such as R16 for R8)
// This is used during the narrowing process because RPi5 PiSP pipeline promotes 8-bit pixels to 16 bit and
// this driver narrows it back to 8-bit
libcamera::PixelFormat get_16bit_variant(const libcamera::PixelFormat& pixelformat);

libcamera::StreamFormats get_common_stream_formats(const libcamera::StreamFormats& formats);
