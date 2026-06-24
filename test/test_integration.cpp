// Integration test with a faked frame source.
//
// We cannot construct LibcameraRosDriver itself: its ctor -> initialize() acquires a real
// camera and exit(1)s without one. So we exercise the *real* per-frame code that the driver
// runs -- detail::fillImageMsg() (message assembly + the assign() copy) -- and push the
// result through a *real* image_transport CameraPublisher/Subscriber, exactly like the
// driver's publish path. Only the camera frame is synthetic.
//
// What this proves end-to-end:
//   - fillImageMsg builds an Image whose width/height/step/encoding/data match the source
//   - the assign() copy is byte-exact (no truncation / no zero-fill corruption)
//   - the image + camera_info travel over the transport and are received together
//   - getNumSubscribers() reflects a real subscriber (the basis of the no-subscriber gate)

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <libcamera_ros_driver/detail/frame_msg.h>

using namespace std::chrono_literals;

namespace
{
  constexpr uint32_t kW = 64;
  constexpr uint32_t kH = 48;
  const std::string kEncoding = "mono8";
}  // namespace

class FrameRoundTrip : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("frame_roundtrip_test");
  }
  rclcpp::Node::SharedPtr node_;
};

TEST_F(FrameRoundTrip, PublishesImageMatchingSource)
{
  image_transport::ImageTransport it(node_);
  auto pub = it.advertiseCamera("image_raw", 1);

  sensor_msgs::msg::Image::ConstSharedPtr got_img;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr got_info;
  auto sub = it.subscribeCamera(
      "image_raw", 1,
      [&](const sensor_msgs::msg::Image::ConstSharedPtr& img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
      {
        got_img = img;
        got_info = info;
      });

  // synthetic frame with a recognizable per-byte pattern
  std::vector<uint8_t> src(static_cast<size_t>(kW) * kH);
  for (size_t i = 0; i < src.size(); ++i)
    src[i] = static_cast<uint8_t>((i * 7 + 1) & 0xFF);

  std_msgs::msg::Header hdr;
  hdr.frame_id = "test_cam";
  hdr.stamp = node_->now();

  // the exact call the driver makes (fd < 0 => skip dmabuf ioctl, plain copy)
  auto built = libcamera_ros_driver::detail::fillImageMsg(hdr, kW, kH, kW, kEncoding, src.data(), src.size(), -1);

  // sanity on the builder output itself before it hits the wire
  ASSERT_EQ(built->width, kW);
  ASSERT_EQ(built->height, kH);
  ASSERT_EQ(built->step, kW);
  ASSERT_EQ(built->encoding, kEncoding);
  ASSERT_EQ(built->data.size(), src.size());
  ASSERT_EQ(built->data, std::vector<uint8_t>(src.begin(), src.end()));

  sensor_msgs::msg::CameraInfo info;
  info.header = hdr;
  info.width = kW;
  info.height = kH;

  // publish repeatedly until discovery completes and the message is delivered (bounded)
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!got_img && std::chrono::steady_clock::now() < deadline)
  {
    pub.publish(*built, info);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(got_img) << "no image received within 5 s (DDS discovery / delivery failed)";
  ASSERT_TRUE(got_info);

  EXPECT_GT(pub.getNumSubscribers(), 0u);  // the no-subscriber gate would (correctly) not fire here
  EXPECT_EQ(got_img->width, kW);
  EXPECT_EQ(got_img->height, kH);
  EXPECT_EQ(got_img->step, kW);
  EXPECT_EQ(got_img->encoding, kEncoding);
  EXPECT_EQ(got_img->data.size(), src.size());
  EXPECT_EQ(got_img->data, std::vector<uint8_t>(src.begin(), src.end()));
  EXPECT_EQ(got_info->width, kW);
}
