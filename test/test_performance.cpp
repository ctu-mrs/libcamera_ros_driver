// Performance gate for the libcamera_ros_driver per-frame work.
//
// The CPU-bound work we control per frame is moving one full frame out of the (mmap'd)
// dmabuf into the outgoing message's std::vector<uint8_t> -- either a plain copy (R16
// passthrough) or, by default (publish_mono8), a MONO16->MONO8 narrowing pass. These tests
// reproduce exactly that memory work in isolation (no camera / ROS node needed, so they run
// deterministically in CI) and assert:
//   1. a full-res plain copy fits comfortably inside the 60 Hz per-frame budget,
//   2. assign() (one uninitialized copy) is no slower than resize()+memcpy(), and
//   3. the MONO16->MONO8 narrowing (the default hot path) also fits the budget -- catches an
//      unvectorized / -O0 build of the inner shift loop.
//
// Budgets are deliberately generous: these gates exist to catch catastrophic regressions
// (e.g. an -O0 build, or reintroducing a double frame-write), not to micro-police
// nanoseconds, so they never flake on a loaded CI box.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <vector>

namespace
{
  // Full-resolution mono frame (~1.5 MB). Larger than the OV9281's 1280x800 so the
  // gate has headroom for bigger sensors / full-res modes.
  constexpr size_t kWidth = 1456;
  constexpr size_t kHeight = 1088;
  constexpr size_t kFrameBytes = kWidth * kHeight;  // 1 byte/px (MONO8)

  // 60 Hz -> 16.667 ms/frame. One copy must be a small fraction of that. 8 ms leaves
  // >20x margin vs a real RPi5 copy (~0.3-0.5 ms) while still failing a build that
  // somehow takes milliseconds per frame.
  constexpr double kFrameBudgetMs = 8.0;

  constexpr int kWarmup = 20;
  constexpr int kIters = 200;

  using Clock = std::chrono::steady_clock;

  double median(std::vector<double>& v)
  {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  }

  // assign(): single uninitialized copy into the message vector (current hot path).
  double time_assign(const uint8_t* src, size_t n)
  {
    const auto t0 = Clock::now();
    std::vector<uint8_t> data;
    data.assign(src, src + n);
    const auto t1 = Clock::now();
    // touch the buffer so the optimizer can't elide the copy
    EXPECT_EQ(data.size(), n);
    EXPECT_EQ(data.back(), src[n - 1]);
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  // resize()+memcpy(): the old path -- zero-fills the whole frame, then overwrites it.
  double time_resize_memcpy(const uint8_t* src, size_t n)
  {
    const auto t0 = Clock::now();
    std::vector<uint8_t> data;
    data.resize(n);
    std::memcpy(data.data(), src, n);
    const auto t1 = Clock::now();
    EXPECT_EQ(data.size(), n);
    EXPECT_EQ(data.back(), src[n - 1]);
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  // MONO16 -> MONO8 narrowing: mirrors the inner loop of detail::fillImageMsgMono8 (the default
  // hot path with publish_mono8). Reads n 16-bit samples, writes n 8-bit, shifting right.
  double time_narrow(const uint16_t* src, uint8_t* dst, size_t n, int shift)
  {
    const auto t0 = Clock::now();
    for (size_t i = 0; i < n; ++i)
      dst[i] = static_cast<uint8_t>(src[i] >> shift);
    const auto t1 = Clock::now();
    EXPECT_EQ(dst[n - 1], static_cast<uint8_t>(src[n - 1] >> shift));  // defeat dead-store elision
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
  }
}  // namespace

class FrameCopyPerf : public ::testing::Test
{
protected:
  void SetUp() override
  {
    src_.resize(kFrameBytes);
    for (size_t i = 0; i < kFrameBytes; ++i)
      src_[i] = static_cast<uint8_t>(i & 0xFF);
  }
  std::vector<uint8_t> src_;
};

// Gate 1: a full-res frame copy fits inside the 60 Hz per-frame budget.
TEST_F(FrameCopyPerf, AssignWithin60HzBudget)
{
  std::vector<double> samples;
  samples.reserve(kIters);
  for (int i = 0; i < kWarmup; ++i)
    time_assign(src_.data(), kFrameBytes);
  for (int i = 0; i < kIters; ++i)
    samples.push_back(time_assign(src_.data(), kFrameBytes));

  const double med = median(samples);
  RecordProperty("median_ms", std::to_string(med));
  EXPECT_LT(med, kFrameBudgetMs)
      << "frame copy median " << med << " ms exceeds " << kFrameBudgetMs
      << " ms budget (60 Hz = 16.67 ms/frame). Likely an unoptimized build.";
}

// Gate 2: assign() is no slower than resize()+memcpy(). The resize path does a strictly
// extra zero-fill pass, so assign should win; 5% tolerance absorbs allocator noise.
TEST_F(FrameCopyPerf, AssignNotSlowerThanResizeMemcpy)
{
  std::vector<double> a, r;
  a.reserve(kIters);
  r.reserve(kIters);
  for (int i = 0; i < kWarmup; ++i)
  {
    time_assign(src_.data(), kFrameBytes);
    time_resize_memcpy(src_.data(), kFrameBytes);
  }
  // interleave to share the same thermal / scheduling conditions
  for (int i = 0; i < kIters; ++i)
  {
    a.push_back(time_assign(src_.data(), kFrameBytes));
    r.push_back(time_resize_memcpy(src_.data(), kFrameBytes));
  }

  const double am = median(a);
  const double rm = median(r);
  RecordProperty("assign_ms", std::to_string(am));
  RecordProperty("resize_memcpy_ms", std::to_string(rm));
  EXPECT_LE(am, rm * 1.05)
      << "assign median " << am << " ms is slower than resize+memcpy " << rm
      << " ms -- the zero-fill-elimination optimization regressed.";
}

// Narrowing perf uses a 16-bit source (2 bytes/px); kFrameBytes is the pixel count.
class Mono8NarrowPerf : public ::testing::Test
{
protected:
  void SetUp() override
  {
    src_.resize(kFrameBytes);  // kFrameBytes pixels of 16-bit input
    dst_.resize(kFrameBytes);
    for (size_t i = 0; i < src_.size(); ++i)
      src_[i] = static_cast<uint16_t>((i & 0x3FF) << 6);  // MSB-aligned 10-bit, as PiSP delivers
  }
  std::vector<uint16_t> src_;
  std::vector<uint8_t> dst_;
};

// Gate 3: a full-res MONO16->MONO8 narrow (the default per-frame work) fits the 60 Hz budget.
TEST_F(Mono8NarrowPerf, NarrowWithin60HzBudget)
{
  std::vector<double> samples;
  samples.reserve(kIters);
  for (int i = 0; i < kWarmup; ++i)
    time_narrow(src_.data(), dst_.data(), src_.size(), 8);
  for (int i = 0; i < kIters; ++i)
    samples.push_back(time_narrow(src_.data(), dst_.data(), src_.size(), 8));

  const double med = median(samples);
  RecordProperty("narrow_median_ms", std::to_string(med));
  EXPECT_LT(med, kFrameBudgetMs)
      << "mono8 narrow median " << med << " ms exceeds " << kFrameBudgetMs
      << " ms budget (60 Hz = 16.67 ms/frame). Likely an unoptimized build of the shift loop.";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
