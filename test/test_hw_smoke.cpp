// Hardware smoke test (opt-in, skipped in CI).
//
// Unlike the other tests, this one needs a real running driver + camera. Enable it by
// launching the driver on the Pi and then running with:
//
//   LIBCAMERA_ROS_HW_TEST=1 \
//   LIBCAMERA_ROS_TOPIC=/uav1/rpi_camera_front/image_raw \
//   LIBCAMERA_ROS_EXPECT_FPS=60 \
//   colcon test --packages-select libcamera_ros_driver \
//     --ctest-args -R test_hw_smoke --event-handlers console_direct+
//
// It subscribes to the live image topic, counts frames over a few seconds, and asserts
// frames actually arrive (and, if EXPECT_FPS is given, that the measured rate is within
// 20% of it). This is the only test that observes the real per-frame pipeline + transport.
//
// NOTE (rmw_zenoh): this test is cross-process, so a Zenoh router must be running for the
// test to discover the driver -- `ros2 run rmw_zenoh_cpp rmw_zenohd` -- otherwise it sees
// zero frames and fails the ASSERT_GT below.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

namespace
{
  const char* env_or(const char* key, const char* fallback)
  {
    const char* v = std::getenv(key);
    return (v && *v) ? v : fallback;
  }
}  // namespace

TEST(HwSmoke, FramesArriveAtExpectedRate)
{
  if (!std::getenv("LIBCAMERA_ROS_HW_TEST"))
    GTEST_SKIP() << "set LIBCAMERA_ROS_HW_TEST=1 (with a camera + running driver) to enable";

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  const std::string topic = env_or("LIBCAMERA_ROS_TOPIC", "image_raw");
  const double duration_s = std::stod(env_or("LIBCAMERA_ROS_DURATION", "5"));

  auto node = std::make_shared<rclcpp::Node>("hw_smoke_test");
  image_transport::ImageTransport it(node);

  size_t count = 0;
  rclcpp::Time first, last;
  auto sub = it.subscribe(topic, 50,
                          [&](const sensor_msgs::msg::Image::ConstSharedPtr& img)
                          {
                            if (count == 0)
                              first = img->header.stamp;
                            last = img->header.stamp;
                            ++count;
                          });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s);
  while (std::chrono::steady_clock::now() < deadline)
    rclcpp::spin_some(node);

  ASSERT_GT(count, 0u) << "no frames on '" << topic << "' -- is the driver running and the topic correct?";

  // measure rate from sensor stamps when we have >=2 frames, else fall back to wall time
  double elapsed = duration_s;
  if (count >= 2 && last > first)
    elapsed = (last - first).seconds() * count / (count - 1);  // span extrapolated to all frames
  const double fps = count / elapsed;

  RecordProperty("frames", std::to_string(count));
  RecordProperty("measured_fps", std::to_string(fps));
  std::cerr << "[hw_smoke] received " << count << " frames on '" << topic << "' -> ~" << fps << " fps\n";

  if (const char* exp = std::getenv("LIBCAMERA_ROS_EXPECT_FPS"))
  {
    const double expected = std::stod(exp);
    EXPECT_NEAR(fps, expected, expected * 0.20) << "measured " << fps << " fps vs expected " << expected << " fps (>20% off)";
  }
}
