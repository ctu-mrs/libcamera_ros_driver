// Integration test with a faked frame source.
//
// We cannot construct LibcameraRosDriver itself: its onInit() acquires a real camera and
// shuts down without one. So we exercise the *real* per-frame builders the driver runs --
// detail::fillImageMsg() (R16 passthrough) and detail::fillImageMsgMono8() (the MONO16->MONO8
// narrowing) -- and push a built message through a *real* image_transport CameraPublisher/
// Subscriber, exactly like the driver's publish path. Only the frame is synthetic.
//
// What this proves:
//   - the builders produce an Image whose width/height/step/encoding/data match the source
//   - the copy / narrow is byte-exact (no truncation, no zero-fill corruption, no stride leak)
//   - shift=8 on MSB-aligned data is correct (the production default; wrong shift = black image)
//   - the image + camera_info travel over the transport and are received together
//   - getNumSubscribers() reflects a real subscriber (the basis of the no-subscriber gate)

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <libcamera_ros_driver/detail/frame_msg.h>

using namespace std::chrono_literals;

namespace
{
  constexpr uint32_t kW = 64;
  constexpr uint32_t kH = 48;
  const std::string kEncoding = "mono8";
}  // namespace

// Pure logic check for the MONO16 -> MONO8 narrowing (no ROS / camera needed). Verifies the
// shift, tight output packing, and that input row stride padding is skipped correctly.
TEST(Mono8Narrowing, ShiftsAndPacks)
{
  constexpr uint32_t w = 4, h = 3;
  constexpr uint32_t stride = 16;  // bytes per input row > w*2, i.e. 8 px slots, last 4 are padding
  constexpr int shift = 2;

  std::vector<uint8_t> raw(static_cast<size_t>(stride) * h, 0xEE);  // padding poison
  for (uint32_t y = 0; y < h; ++y)
  {
    uint16_t* row = reinterpret_cast<uint16_t*>(raw.data() + static_cast<size_t>(y) * stride);
    for (uint32_t x = 0; x < w; ++x)
      row[x] = static_cast<uint16_t>((y * w + x) << shift);  // so >>shift recovers the index
  }

  std_msgs::Header hdr;
  auto msg = libcamera_ros_driver::detail::fillImageMsgMono8(hdr, w, h, stride, raw.data(), -1, shift);

  ASSERT_EQ(msg.encoding, "mono8");
  ASSERT_EQ(msg.step, w);
  ASSERT_EQ(msg.data.size(), static_cast<size_t>(w) * h);
  for (uint32_t i = 0; i < w * h; ++i)
    EXPECT_EQ(msg.data[i], static_cast<uint8_t>(i)) << "pixel " << i << " (stride padding leaked?)";
}

// Production default: PiSP delivers each sample MSB-aligned (10-bit value << 6 in a 16-bit
// word), so shift=8 takes the top byte. This is the exact case where shift=2 produced an
// all-black image -- guard it so that regression can't return.
TEST(Mono8Narrowing, Shift8TakesTopByteMsbAligned)
{
  constexpr uint32_t w = 8, h = 2;
  constexpr uint32_t stride = w * 2;  // tight, 2 bytes/px
  std::vector<uint8_t> raw(static_cast<size_t>(stride) * h);
  for (uint32_t i = 0; i < w * h; ++i)
  {
    const uint16_t v10 = static_cast<uint16_t>((i * 67) & 0x3FF);            // a 10-bit value
    reinterpret_cast<uint16_t*>(raw.data())[i] = static_cast<uint16_t>(v10 << 6);  // MSB-aligned
  }

  std_msgs::Header hdr;
  auto msg = libcamera_ros_driver::detail::fillImageMsgMono8(hdr, w, h, stride, raw.data(), -1, 8);

  ASSERT_EQ(msg.data.size(), static_cast<size_t>(w) * h);
  for (uint32_t i = 0; i < w * h; ++i)
  {
    const uint16_t v10 = static_cast<uint16_t>((i * 67) & 0x3FF);
    const uint8_t expected = static_cast<uint8_t>((v10 << 6) >> 8);  // top 8 bits = v10 >> 2
    EXPECT_EQ(msg.data[i], expected) << "pixel " << i << " (wrong shift -> black image)";
  }
}

class FrameRoundTrip : public ::testing::Test
{
protected:
  void SetUp() override
  {
    nh_.reset(new ros::NodeHandle("~"));
  }
  std::unique_ptr<ros::NodeHandle> nh_;
};

TEST_F(FrameRoundTrip, PublishesImageMatchingSource)
{
  image_transport::ImageTransport it(*nh_);
  auto pub = it.advertiseCamera("image_raw", 1);

  sensor_msgs::ImageConstPtr got_img;
  sensor_msgs::CameraInfoConstPtr got_info;
  auto sub = it.subscribeCamera(
      "image_raw", 1,
      boost::function<void(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&)>(
          [&](const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info)
          {
            got_img = img;
            got_info = info;
          }));

  // synthetic frame with a recognizable per-byte pattern
  std::vector<uint8_t> src(static_cast<size_t>(kW) * kH);
  for (size_t i = 0; i < src.size(); ++i)
    src[i] = static_cast<uint8_t>((i * 7 + 1) & 0xFF);

  std_msgs::Header hdr;
  hdr.frame_id = "test_cam";
  hdr.stamp = ros::Time::now();

  // the exact call the driver makes (fd < 0 => skip dmabuf ioctl, plain copy)
  auto built = libcamera_ros_driver::detail::fillImageMsg(hdr, kW, kH, kW, kEncoding, src.data(), src.size(), -1);

  // sanity on the builder output itself before it hits the wire
  ASSERT_EQ(built.width, kW);
  ASSERT_EQ(built.height, kH);
  ASSERT_EQ(built.step, kW);
  ASSERT_EQ(built.encoding, kEncoding);
  ASSERT_EQ(built.data.size(), src.size());
  ASSERT_EQ(built.data, std::vector<uint8_t>(src.begin(), src.end()));

  sensor_msgs::CameraInfo info;
  info.header = hdr;
  info.width = kW;
  info.height = kH;

  // publish repeatedly until discovery completes and the message is delivered (bounded)
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!got_img && std::chrono::steady_clock::now() < deadline)
  {
    pub.publish(built, info);
    ros::spinOnce();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(got_img) << "no image received within 5 s (pub/sub delivery failed)";
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "frame_roundtrip_test");
  return RUN_ALL_TESTS();
}
