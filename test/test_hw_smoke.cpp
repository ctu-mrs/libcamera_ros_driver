// Hardware smoke test (opt-in, skipped in CI).
//
// Unlike the other tests, this one needs a real running driver + camera. Enable it by
// launching the driver on the Pi and then running with these env vars set:
//
//   LIBCAMERA_ROS_HW_TEST=1
//   LIBCAMERA_ROS_TOPIC=/uav1/camera_front/image_raw
//   LIBCAMERA_ROS_EXPECT_FPS=60
//   catkin run_tests libcamera_ros_driver --no-deps
//
// It subscribes to the live image topic, counts frames over a few seconds, and asserts
// frames actually arrive (and, if EXPECT_FPS is given, that the measured rate is within
// 20% of it). This is the only test that observes the real per-frame pipeline + transport.
//
// The skip check runs BEFORE any ROS call, so the test is safe to build/run without a master.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

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

  if (!ros::isInitialized())
  {
    int argc = 0;
    ros::init(argc, nullptr, "hw_smoke_test");
  }

  const std::string topic = env_or("LIBCAMERA_ROS_TOPIC", "image_raw");
  const double duration_s = std::stod(env_or("LIBCAMERA_ROS_DURATION", "5"));

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  size_t count = 0;
  ros::Time first, last;
  auto sub = it.subscribe(topic, 50,
                          boost::function<void(const sensor_msgs::ImageConstPtr&)>(
                              [&](const sensor_msgs::ImageConstPtr& img)
                              {
                                if (count == 0)
                                  first = img->header.stamp;
                                last = img->header.stamp;
                                ++count;
                              }));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s);
  while (std::chrono::steady_clock::now() < deadline)
    ros::spinOnce();

  ASSERT_GT(count, 0u) << "no frames on '" << topic << "' -- is the driver running and the topic correct?";

  // measure rate from sensor stamps when we have >=2 frames, else fall back to wall time
  double elapsed = duration_s;
  if (count >= 2 && last > first)
    elapsed = (last - first).toSec() * count / (count - 1);  // span extrapolated to all frames
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
