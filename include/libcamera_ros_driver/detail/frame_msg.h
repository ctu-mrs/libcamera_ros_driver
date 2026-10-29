#pragma once

// Per-frame message assembly, factored out of LibcameraRosDriver::requestComplete so it
// can be unit/integration tested without a camera. Header-only + inline so the driver's
// hot path keeps the exact same cost (the compiler inlines it back in).

#include <cstdint>
#include <memory>
#include <string>

#include <sys/ioctl.h>
#include <linux/dma-buf.h>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

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

  // Build an 8-bit Image by narrowing a 16-bit frame down to 8 bits, fused into the one
  // unavoidable copy. Halves the published payload (and thus the serialization + transport +
  // subscriber copy) vs shipping the full 16-bit container. `src_stride` is the input row stride
  // in BYTES (may exceed width*2 due to alignment); the output is tightly packed at 1 byte/pixel.
  inline std::unique_ptr<sensor_msgs::msg::Image> fillImageMsgNarrow8(const std_msgs::msg::Header& hdr, uint32_t width, uint32_t height, uint32_t src_stride,
                                                                      const std::string& encoding, const uint8_t* src, int fd)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = hdr;
    msg->width = width;
    msg->height = height;
    msg->step = width; // 1 byte/pixel, tightly packed
    msg->encoding = encoding;
    msg->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);

    // ponytail: resize zero-fills first, but every byte is overwritten below, so the cost is a
    // single 1 MB memset -- negligible next to halving the downstream serialize/transport.
    msg->data.resize(static_cast<size_t>(width) * height);

    if (fd >= 0)
    {
      dma_buf_sync sync_start = {DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
      ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync_start);
    }
    for (uint32_t y = 0; y < height; ++y)
    {
      const uint16_t* in = reinterpret_cast<const uint16_t*>(src + static_cast<size_t>(y) * src_stride);
      uint8_t* out = msg->data.data() + static_cast<size_t>(y) * width;
      uint32_t x = 0;
      // If we have NEON support use it. O3 optimization would use it automatically, but this makes sure we use it even with -O2 or lower
#if defined(__ARM_NEON) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      // little-endian: the odd bytes are the high bytes
      for (; x + 16 <= width; x += 16)
        vst1q_u8(out + x, vld2q_u8(reinterpret_cast<const uint8_t*>(in + x)).val[1]);
#endif
      for (; x < width; ++x)
        out[x] = static_cast<uint8_t>(in[x] >> 8);
    }
    if (fd >= 0)
    {
      dma_buf_sync sync_end = {DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ};
      ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync_end);
    }

    return msg;
  }

} // namespace libcamera_ros_driver::detail
