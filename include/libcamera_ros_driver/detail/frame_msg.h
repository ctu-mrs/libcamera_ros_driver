#pragma once

// Per-frame message assembly, factored out of LibcameraRosDriver::requestComplete so it
// can be unit/integration tested without a camera. Header-only + inline so the driver's
// hot path keeps the exact same cost (the compiler inlines it back in).

#include <cstdint>
#include <memory>
#include <string>

#include <sys/ioctl.h>
#include <linux/dma-buf.h>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace libcamera_ros_driver::detail
{

  // Build the outgoing Image from a raw frame buffer.
  //
  //  - assign() does a single uninitialized copy (resize()+memcpy would zero-fill then
  //    overwrite -- two passes over the frame).
  //  - fd >= 0 wraps the copy in a dmabuf cache invalidate/flush so a CPU read of a cached
  //    capture buffer is correct and full-speed. Pass fd < 0 for coherent buffers or tests.
  inline std::unique_ptr<sensor_msgs::msg::Image> fillImageMsg(const std_msgs::msg::Header& hdr, uint32_t width, uint32_t height, uint32_t step,
                                                               const std::string& encoding, const uint8_t* src, size_t size, int fd)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = hdr;
    msg->width = width;
    msg->height = height;
    msg->step = step;
    msg->encoding = encoding;
    msg->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);

    if (fd >= 0)
    {
      dma_buf_sync sync_start = {DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
      ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync_start);
    }
    msg->data.assign(src, src + size);
    if (fd >= 0)
    {
      dma_buf_sync sync_end = {DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ};
      ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync_end);
    }

    return msg;
  }

}  // namespace libcamera_ros_driver::detail
