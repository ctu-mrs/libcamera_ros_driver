#pragma once
#include <ostream>

namespace libcamera
{
  class CameraManager;
  class StreamFormats;
  struct StreamConfiguration;

  std::ostream& operator<<(std::ostream& out, const CameraManager& camera_manager);
  std::ostream& operator<<(std::ostream& out, const StreamFormats& formats);
  std::ostream& operator<<(std::ostream& out, const StreamConfiguration& configuration);

} // namespace libcamera
