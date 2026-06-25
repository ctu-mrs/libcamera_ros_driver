/* includes //{ */

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <thread>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libcamera_ros/libcamera_ros.h>

#include <libcamera_ros_driver/utils/clamp.h>
#include <libcamera_ros_driver/utils/format_mapping.h>
#include <libcamera_ros_driver/utils/stream_mapping.h>
#include <libcamera_ros_driver/utils/control_mapping.h>
#include <libcamera_ros_driver/utils/pretty_print.h>
#include <libcamera_ros_driver/utils/type_extent.h>
#include <libcamera_ros_driver/utils/types.h>
#include <libcamera_ros_driver/utils/pv_to_cv.h>
#include <libcamera_ros_driver/utils/is_vector.h>
#include <libcamera_ros_driver/detail/frame_msg.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//}

namespace libcamera_ros_driver
{

/* getCameraManager() //{ */

// libcamera forbids more than one CameraManager per process (CameraManager ctor
// calls LOG(Camera, Fatal) -> abort if a second is created). To run several camera
// nodelets in one nodelet manager (e.g. a true stereo pair sharing a process for
// coherent timestamps), they must share a single manager. This returns a process-wide
// instance, started once on first use and stopped (via ~CameraManager) when the last
// nodelet releases its shared_ptr.
// ponytail: weak_ptr + mutex, the simplest correct lifetime. Upgrade only if start/stop
// ever needs to be decoupled from nodelet lifetime.
static std::shared_ptr<libcamera::CameraManager> getCameraManager() {

  static std::mutex                              mtx;
  static std::weak_ptr<libcamera::CameraManager> weak;

  std::scoped_lock lock(mtx);

  if (auto cm = weak.lock()) {
    return cm;
  }

  auto cm = std::make_shared<libcamera::CameraManager>();
  if (cm->start() != 0) {
    throw std::runtime_error("Failed to start libcamera CameraManager");
  }

  weak = cm;
  return cm;
}

//}

/* ParamCheck() method //{ */
template <typename T>
bool getOptionalParamCheck(const ros::NodeHandle &nh, const std::string &node_name, const std::string &param_name, T &param_out) {

  if (!nh.getParam(param_name, param_out)) {
    return false;
  }

  ROS_INFO_STREAM("[LibcameraRosDriver]: " << param_name << "': " << param_out);

  return true;
}

template <typename T>
bool getCompulsoryParamCheck(const ros::NodeHandle &nh, const std::string &node_name, const std::string &param_name, T &param_out) {

  const bool res = nh.getParam(param_name, param_out);

  if (!res) {
    ROS_ERROR_STREAM("[LibcameraRosDriver]: Could not load compulsory parameter '" << param_name << "'");
  } else {
    ROS_INFO_STREAM("[LibcameraRosDriver]: Loaded parameter " << param_name << "': " << param_out);
  }

  return res;
}

template <typename T>
bool getCompulsoryParamCheck(const ros::NodeHandle &nh, const std::string &node_name, const std::string &param_name, T &param_out, const T &param_default) {

  const bool res = nh.getParam(param_name, param_out);

  if (!res) {
    param_out = param_default;
  }

  ROS_INFO_STREAM("[LibcameraRosDriver]: Loaded parameter '" << param_name << "': " << param_out);

  return res;
}

//}

/* class LibcameraRosDriver //{ */

class LibcameraRosDriver : public nodelet::Nodelet {
public:
  virtual void onInit();
  ~LibcameraRosDriver();

private:
  ros::NodeHandle nh_;

  std::shared_ptr<libcamera::CameraManager>        camera_manager_;
  std::shared_ptr<libcamera::Camera>               camera_;
  libcamera::Stream *                              stream_;
  std::shared_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::mutex                                       request_lock_;

  std::string frame_id_;

  // per-frame-constant image properties, cached at init to keep the callback free of map
  // lookups and a per-frame std::string heap allocation
  bool        is_raw_ = false;
  std::string encoding_;
  uint32_t    img_width_  = 0;
  uint32_t    img_height_ = 0;
  uint32_t    img_step_   = 0;
  uint32_t    src_stride_ = 0;  // input row stride in bytes (for mono16->mono8 narrowing)

  // opt-in: publish MONO8 by narrowing a MONO16 frame. Halves the serialized payload for
  // consumers (e.g. VIO) that only use 8-bit greyscale. Default off keeps the raw R16 output.
  bool mono8_       = false;
  int  mono8_shift_ = 8;  // PiSP unpacks raw MSB-aligned, so the top byte (>>8) is the image

  // dmabuf cache invalidate/flush around the CPU read of each frame. Necessary for correct
  // reads of NON-coherent capture buffers; pure overhead if the buffers are already coherent.
  // ponytail: default ON = always correct. Turn off only after confirming images stay intact.
  bool dmabuf_sync_ = true;

  bool _use_ros_time_ = false;

  ros::Duration start_time_offset_;
  bool          start_time_offset_obtained_ = false;

  struct buffer_info_t
  {
    void * data;
    size_t size;
    int    fd;  // dmabuf fd, for cache-sync around CPU reads
  };
  std::unordered_map<const libcamera::FrameBuffer *, buffer_info_t> buffer_info_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  // ponytail: cached once at init to avoid a per-frame copy + mutex inside the callback;
  // does not reflect runtime recalibration via the set_camera_info service. Refresh on that service if ever needed.
  sensor_msgs::CameraInfo cinfo_msg_;

  image_transport::CameraPublisher image_pub_;

  // Frame offload: the libcamera requestCompleted callback runs on the CameraManager's single
  // (per-process, shared in stereo) thread, so doing the per-frame work there serializes both
  // cameras onto one core. We hand the RAW capture buffer to a per-nodelet worker thread, which
  // does the copy + mono8 narrow + serialize + publish off the camera thread (parallel across
  // cameras) and then re-queues the request -- the buffer must stay intact until it has copied.
  // ponytail: single-slot latest-frame-wins. A queue would only add latency for a live camera;
  // a superseded frame's request is re-queued immediately so its buffer is never leaked.
  std::thread             publish_thread_;
  std::mutex              publish_mutex_;
  std::condition_variable publish_cv_;
  struct PendingFrame
  {
    libcamera::Request * request = nullptr;  // re-queued by the worker once the buffer is copied
    const uint8_t *      data    = nullptr;
    size_t               size    = 0;
    int                  fd      = -1;  // dmabuf fd for cache-sync, or -1 to skip
    std_msgs::Header     hdr;
  };
  PendingFrame pending_;
  bool         publish_stop_ = false;
  void         publishLoop();

  // map parameter names to libcamera control id
  std::unordered_map<std::string, const libcamera::ControlId *> parameter_ids_;
  // parameters that are to be set for every request
  std::unordered_map<unsigned int, libcamera::ControlValue> parameters_;

  void declareControlParameters();
  void requestComplete(libcamera::Request *request);

  bool updateControlParameter(const libcamera::ControlValue &value, const libcamera::ControlId *id);
};

//}

/* LibcameraRosDriver::onInit method //{ */

void LibcameraRosDriver::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters //{ */

  bool        success = true;
  std::string camera_name;
  std::string stream_role;
  std::string pixel_format;
  std::string calib_url;
  int         camera_id;
  int         resolution_width;
  int         resolution_height;

  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "camera_name", camera_name);
  success = success && getOptionalParamCheck(nh_, "LibcameraRosDriver", "camera_id", camera_id);
  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "stream_role", stream_role);
  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "pixel_format", pixel_format);
  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "frame_id", frame_id_);
  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "calib_url", calib_url);
  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "resolution/width", resolution_width);
  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "resolution/height", resolution_height);
  success = success && getCompulsoryParamCheck(nh_, "LibcameraRosDriver", "use_ros_time", _use_ros_time_);

  if (!success) {
    ROS_ERROR("[LibcameraRosDriver]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
    return;
  }

  // optional performance / format parameters (defaults are the member initializers)
  getOptionalParamCheck(nh_, "LibcameraRosDriver", "publish_mono8", mono8_);
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "mono8_shift", mono8_shift_)) {
    mono8_shift_ = std::clamp(mono8_shift_, 0, 15);  // a uint16 shift outside [0,15] is UB
  }
  getOptionalParamCheck(nh_, "LibcameraRosDriver", "dmabuf_sync", dmabuf_sync_);

  //}

  // start (or join) the process-wide camera manager and check for cameras
  camera_manager_ = getCameraManager();
  if (camera_manager_->cameras().empty()) {
    ROS_ERROR("[LibcameraRosDriver]: no cameras available");
    ros::shutdown();
    return;
  }

  if (!camera_name.empty()) {
    std::vector<std::string> available_cameras;
    ROS_INFO_STREAM("[LibcameraRosDriver]: Available cameras:");
    for (size_t i = 0; i < camera_manager_->cameras().size(); i++) {
      available_cameras.push_back(camera_manager_->cameras().at(i)->id());
    }
    for (size_t i = 0; i < available_cameras.size(); i++) {
      if (available_cameras.at(i).find(camera_name) != std::string::npos) {
        ROS_INFO_STREAM("[LibcameraRosDriver]: found camera: " << camera_name << " index: " << i << " at: " << available_cameras.at(i));
        camera_id = i;
        break;
      }
    }
  }

  if (camera_id >= int(camera_manager_->cameras().size())) {
    ROS_INFO_STREAM(*camera_manager_);
    ROS_ERROR_STREAM("[LibcameraRosDriver]: camera with id " << camera_name << " does not exist");
    ros::shutdown();
    return;
  }
  camera_ = camera_manager_->cameras().at(camera_id);
  ROS_INFO_STREAM("[LibcameraRosDriver]: Use camera by id: " << camera_id);

  if (!camera_) {
    ROS_INFO_STREAM("[LibcameraRosDriver]: " << camera_manager_);
    ROS_ERROR_STREAM("[LibcameraRosDriver]: "
                     << "camera with name " << camera_name << " does not exist");
    ros::shutdown();
    return;
  }

  if (camera_->acquire()) {
    ROS_ERROR("[LibcameraRosDriver]: failed to acquire camera");
    ros::shutdown();
    return;
  }

  // configure camera stream
  std::unique_ptr<libcamera::CameraConfiguration> cfg = camera_->generateConfiguration({get_role(stream_role)});

  if (!cfg) {
    ROS_ERROR("[LibcameraRosDriver]: failed to generate configuration");
    ros::shutdown();
    return;
  }

  libcamera::StreamConfiguration &scfg = cfg->at(0);

  // get common pixel formats that are supported by the camera and the node
  const libcamera::StreamFormats            stream_formats = get_common_stream_formats(scfg.formats());
  const std::vector<libcamera::PixelFormat> common_fmt     = stream_formats.pixelformats();

  if (common_fmt.empty()) {
    ROS_ERROR("[LibcameraRosDriver]: camera does not provide any of the supported pixel formats");
    ros::shutdown();
    return;
  }

  if (pixel_format.empty()) {

    // auto select first common pixel format
    scfg.pixelFormat = common_fmt.front();  // get pixel format from provided string
    ROS_INFO_STREAM("[LibcameraRosDriver]: " << stream_formats);
    ROS_WARN_STREAM("[LibcameraRosDriver]: no pixel format selected, using default: \"" << scfg.pixelFormat << "\"");
    ROS_WARN_STREAM("[LibcameraRosDriver]: set parameter 'pixel_format' to silent this warning");
  } else {

    // get pixel format from provided string
    const libcamera::PixelFormat format_requested = libcamera::PixelFormat::fromString(pixel_format);

    if (!format_requested.isValid()) {
      ROS_INFO_STREAM("[LibcameraRosDriver]: " << stream_formats);
      ROS_ERROR_STREAM("[LibcameraRosDriver]: invalid pixel format: \"" << pixel_format << "\"");
      ros::shutdown();
      return;
    }

    // check that the requested format is supported by camera and the node
    if (std::find(common_fmt.begin(), common_fmt.end(), format_requested) == common_fmt.end()) {
      ROS_INFO_STREAM("[LibcameraRosDriver]: " << stream_formats);
      ROS_ERROR_STREAM("[LibcameraRosDriver]: unsupported pixel format \"" << pixel_format << "\"");
      ros::shutdown();
      return;
    }

    scfg.pixelFormat = format_requested;
  }

  const libcamera::Size size(resolution_width, resolution_height);

  if (size.isNull()) {
    ROS_INFO_STREAM(scfg);
    scfg.size = scfg.formats().sizes(scfg.pixelFormat).back();
    ROS_WARN_STREAM("[LibcameraRosDriver]: no dimensions selected, auto-selecting: \"" << scfg.size << "\"");
    ROS_WARN_STREAM("[LibcameraRosDriver]: set parameters 'resolution/width' and 'resolution/height' to silent this warning");
  } else {
    scfg.size = size;
  }

  // The StillCapture/Raw roles default to a very low buffer count on the RPi pipeline
  // (often 1). With so few buffers the sensor stalls the moment our callback is still
  // holding one, capping the rate far below the requested fps. Give the pipeline enough
  // in-flight buffers to keep capturing while a frame is copied/published.
  // ponytail: 4 is the RPi viewfinder/video default; raise it only if drops persist.
  scfg.bufferCount = std::max<unsigned int>(scfg.bufferCount, 4);

  // store selected stream configuration
  const libcamera::StreamConfiguration selected_scfg = scfg;

  switch (cfg->validate()) {

    case libcamera::CameraConfiguration::Valid: {
      break;
    }

    case libcamera::CameraConfiguration::Adjusted: {

      if (selected_scfg.pixelFormat != scfg.pixelFormat) {
        ROS_INFO_STREAM(stream_formats);
      }

      if (selected_scfg.size != scfg.size) {
        ROS_INFO_STREAM(scfg);
      }

      ROS_WARN_STREAM("[LibcameraRosDriver]: stream configuration adjusted from \"" << selected_scfg.toString() << "\" to \"" << scfg.toString() << "\"");

      break;
    }

    case libcamera::CameraConfiguration::Invalid: {

      ROS_ERROR("[LibcameraRosDriver]: failed to valid stream configuration");
      ros::shutdown();

      return;
      break;
    }
  }

  if (camera_->configure(cfg.get()) < 0) {
    ROS_ERROR("[LibcameraRosDriver]: failed to configure streams");
    ros::shutdown();
    return;
  }

  ROS_INFO_STREAM("[LibcameraRosDriver]: camera \"" << camera_->id() << "\" configured with " << scfg.toString() << " stream");

  // cache per-frame-constant image properties (format/size are fixed after configure)
  is_raw_     = (format_type(scfg.pixelFormat) == FormatType::RAW);
  encoding_   = get_ros_encoding(scfg.pixelFormat);
  img_width_  = scfg.size.width;
  img_height_ = scfg.size.height;
  src_stride_ = scfg.stride;
  img_step_   = scfg.stride;

  // mono8 narrowing only applies to a 16-bit mono source; otherwise fall back to passthrough
  if (mono8_ && encoding_ == "mono16") {
    encoding_ = "mono8";
    img_step_ = img_width_;  // 1 byte/pixel, tightly packed
    ROS_INFO_STREAM("[LibcameraRosDriver]: publish_mono8: narrowing MONO16 -> MONO8 (shift " << mono8_shift_ << ")");
  } else if (mono8_) {
    mono8_ = false;
    ROS_WARN_STREAM("[LibcameraRosDriver]: publish_mono8 requested but source encoding is '" << encoding_ << "', not mono16; publishing unchanged");
  }

  if (!dmabuf_sync_) {
    ROS_WARN("[LibcameraRosDriver]: dmabuf_sync disabled: skipping cache invalidate around frame reads. "
             "Verify images are intact -- non-coherent buffers can read stale/torn data.");
  }

  declareControlParameters();

  int              param_int;
  float            param_float;
  std::string      param_string;
  bool             param_bool;
  std::vector<int> param_vector_int;

  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/exposure_time", param_int)) {
    updateControlParameter(pv_to_cv(param_int, parameter_ids_["ExposureTime"]->type()), parameter_ids_["ExposureTime"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/fps", param_float)) {
    int64_t frame_time = 1000000 / param_float;
    updateControlParameter(pv_to_cv(std::vector<int64_t>{frame_time, frame_time}, parameter_ids_["FrameDurationLimits"]->type()),
                           parameter_ids_["FrameDurationLimits"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/ae_constraint_mode", param_string)) {
    updateControlParameter(pv_to_cv(get_ae_constraint_mode(param_string), parameter_ids_["AeConstraintMode"]->type()), parameter_ids_["AeConstraintMode"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/brightness", param_float)) {
    updateControlParameter(pv_to_cv(param_float, parameter_ids_["Brightness"]->type()), parameter_ids_["Brightness"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/sharpness", param_float)) {
    updateControlParameter(pv_to_cv(param_float, parameter_ids_["Sharpness"]->type()), parameter_ids_["Sharpness"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/awb_enable", param_bool)) {
    if (parameter_ids_["AwbEnable"])  // if the parameter is set when not available, we would get a segmentation fault upon extracting its ->type()
      updateControlParameter(pv_to_cv(param_bool, parameter_ids_["AwbEnable"]->type()), parameter_ids_["AwbEnable"]);
    else
      ROS_ERROR_STREAM("[LibcameraRosDriver]: Parameter AwbEnable is not available! Maybe the selected camera is grayscale");
  }
  /* updateControlParameter<std::vector<float>>(std::string("control/colour_gains"), parameter_ids_["ColourGains"]); */
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/ae_enable", param_bool)) {
    updateControlParameter(pv_to_cv(param_bool, parameter_ids_["AeEnable"]->type()), parameter_ids_["AeEnable"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/saturation", param_float)) {
    updateControlParameter(pv_to_cv(param_float, parameter_ids_["Saturation"]->type()), parameter_ids_["Saturation"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/contrast", param_float)) {
    updateControlParameter(pv_to_cv(param_float, parameter_ids_["Contrast"]->type()), parameter_ids_["Contrast"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/exposure_value", param_float)) {
    updateControlParameter(pv_to_cv(param_float, parameter_ids_["ExposureValue"]->type()), parameter_ids_["ExposureValue"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/analogue_gain", param_float)) {
    updateControlParameter(pv_to_cv(param_float, parameter_ids_["AnalogueGain"]->type()), parameter_ids_["AnalogueGain"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/awb_mode", param_string)) {
    updateControlParameter(pv_to_cv(get_awb_mode(param_string), parameter_ids_["AwbMode"]->type()), parameter_ids_["AwbMode"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/ae_metering_mode", param_string)) {
    updateControlParameter(pv_to_cv(get_ae_metering_mode(param_string), parameter_ids_["AeMeteringMode"]->type()), parameter_ids_["AeMeteringMode"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/scaler_crop", param_vector_int)) {
    updateControlParameter(pv_to_cv(std::vector<int64_t>{param_vector_int.begin(), param_vector_int.end()}, parameter_ids_["ScalerCrop"]->type()),
                           parameter_ids_["ScalerCrop"]);
  }
  if (getOptionalParamCheck(nh_, "LibcameraRosDriver", "control/control", param_string)) {
    updateControlParameter(pv_to_cv(get_ae_exposure_mode(param_string), parameter_ids_["AeExposureMode"]->type()), parameter_ids_["AeExposureMode"]);
  }

  // allocate stream buffers and create one request per buffer
  stream_ = scfg.stream();

  allocator_ = std::make_shared<libcamera::FrameBufferAllocator>(camera_);
  allocator_->allocate(stream_);

  for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator_->buffers(stream_)) {

    std::unique_ptr<libcamera::Request> request = camera_->createRequest();

    if (!request) {
      ROS_ERROR("[LibcameraRosDriver]: Can't create request");
      ros::shutdown();
      return;
    }

    // multiple planes of the same buffer use the same file descriptor
    size_t buffer_length = 0;
    int    fd            = -1;
    for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {

      if (plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset) {
        ROS_ERROR("[LibcameraRosDriver]: invalid offset");
        ros::shutdown();
        return;
      }

      buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);

      if (!plane.fd.isValid()) {
        ROS_ERROR("[LibcameraRosDriver]: file descriptor is not valid");
        ros::shutdown();
        return;
      }

      if (fd == -1) {
        fd = plane.fd.get();
      } else if (fd != plane.fd.get()) {
        ROS_ERROR("[LibcameraRosDriver]: plane file descriptors differ");
        ros::shutdown();
        return;
      }
    }

    // memory-map the frame buffer planes
    void *data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);

    if (data == MAP_FAILED) {
      ROS_ERROR_STREAM("[LibcameraRosDriver]: mmap failed: " << std::string(std::strerror(errno)));
      ros::shutdown();
      return;
    }

    buffer_info_[buffer.get()] = {data, buffer_length, fd};

    if (request->addBuffer(stream_, buffer.get()) < 0) {
      ROS_ERROR("[LibcameraRosDriver]: Can't set buffer for request");
      ros::shutdown();
      return;
    }

    // set modified control parameters
    for (const auto &[id, value] : parameters_) {
      request->controls().set(id, value);
    }

    requests_.push_back(std::move(request));
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh_, camera_name, calib_url);
  cinfo_msg_ = cinfo_->getCameraInfo();

  /* initialize publishers //{ */

  image_transport::ImageTransport it(nh_);
  image_pub_ = it.advertiseCamera("image_raw", 1);

  //}

  // register callback
  camera_->requestCompleted.connect(this, &LibcameraRosDriver::requestComplete);

  // start camera and queue all requests
  if (camera_->start()) {
    ROS_ERROR("[LibcameraRosDriver]: failed to start camera");
    ros::shutdown();
    return;
  }

  // start the publish worker before frames begin arriving
  publish_thread_ = std::thread(&LibcameraRosDriver::publishLoop, this);

  for (std::unique_ptr<libcamera::Request> &request : requests_) {
    camera_->queueRequest(request.get());
  }

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[LibcameraRosDriver]: initialized");
}
//}

/* LibcameraRosDriver::~LibcameraRosDriver() //{ */

LibcameraRosDriver::~LibcameraRosDriver() {

  camera_->requestCompleted.disconnect();  // no more frames handed off after this

  // Stop the worker BEFORE the camera: the worker calls queueRequest, so it must be done
  // before we stop the camera. Any frame still pending is dropped; camera stop reclaims it.
  {
    std::scoped_lock lock(publish_mutex_);
    publish_stop_ = true;
  }
  publish_cv_.notify_one();
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }

  {
    std::scoped_lock lock(request_lock_);

    if (camera_->stop()) {
      ROS_ERROR("[LibcameraRosDriver]: failed to stop camera");
    }
  }

  camera_->release();
  // Do not stop the manager here: it is shared. Dropping our shared_ptr (member
  // destruction) stops it only once the last camera nodelet is gone.

  for (const auto &e : buffer_info_) {
    if (munmap(e.second.data, e.second.size) == -1) {
      ROS_ERROR_STREAM("[LibcameraRosDriver]: "
                       << "munmap failed: " << std::strerror(errno));
    }
  }
}

//}

/* LibcameraRosDriver::declareControlParameters() //{ */

void LibcameraRosDriver::declareControlParameters() {

  ROS_INFO("[LibcameraRosDriver]: available control parameters:");

  for (const auto &[id, info] : camera_->controls()) {

    std::size_t extent;
    try {
      extent = get_extent(id);
    }
    catch (const std::runtime_error &e) {
      // ignore
      ROS_INFO_STREAM("[LibcameraRosDriver]:     " << id->name() << " : Not handled by the current version of the libcamera SDK");
      continue;
    }

    // store control id with name
    parameter_ids_[id->name()] = id;

    if (info.min().numElements() != info.max().numElements()) {
      ROS_ERROR("[LibcameraRosDriver]: minimum and maximum parameter array sizes do not match");
      ros::shutdown();
      return;
    }

    ROS_INFO_STREAM("[LibcameraRosDriver]:     " << id->name() << " : " << info.toString()
                                                 << (info.def().isNone() ? "" : " (default: {" + info.def().toString() + "})"));
  }
}

//}

/* LibcameraRosDriver::updateControlParameter() //{ */

bool LibcameraRosDriver::updateControlParameter(const libcamera::ControlValue &value, const libcamera::ControlId *id) {

  if (value.isNone()) {
    ROS_ERROR_STREAM("[LibcameraRosDriver]: " << id->name().c_str() << " : parameter type not defined");
    return false;
  }

  // verify parameter type and dimension against default
  const libcamera::ControlInfo &ci = camera_->controls().at(id);

  if (value.type() != id->type()) {
    ROS_ERROR_STREAM("[LibcameraRosDriver]: " << id->name().c_str() << " : parameter types mismatch, expected '" << std::to_string(id->type()).c_str()
                                              << "', got '" << std::to_string(value.type()).c_str() << "'");
    return false;
  }

  const std::size_t extent = get_extent(id);
  if ((value.isArray() && (extent > 0)) && value.numElements() != extent) {
    ROS_ERROR_STREAM("[LibcameraRosDriver]: " << id->name().c_str() << " : parameter dimensions mismatch, expected " << std::to_string(extent).c_str()
                                              << ", got " << std::to_string(value.numElements()).c_str());
    return false;
  }

  // check bounds and return error
  // it seems that for exposition the 0 is used for maximum value, which means infinity
  // therefore, we are checking if max > min. If yes, check  if the value is lower than max.
  if (value < ci.min() || (ci.max() > ci.min() ? value > ci.max() : false)) {
    ROS_ERROR_STREAM("[LibcameraRosDriver]: " << id->name().c_str() << " : parameter value " << value.toString().c_str()
                                              << " outside of range: " << ci.toString().c_str());
    return false;
  }

  parameters_[id->id()] = value;

  return true;
}

//}

/* LibcameraRosDriver::requestComplete() //{ */

void LibcameraRosDriver::requestComplete(libcamera::Request *request) {

  std::scoped_lock lock(request_lock_);

  // If we hand the buffer to the worker, the worker re-queues the request (after it has copied
  // out of the buffer); we must NOT re-queue here or the buffer would be reused mid-read. Every
  // other path re-queues below, so a request can never leave without going back to the pool --
  // a leaked request drains the finite pool and silently stops the camera forever.
  bool                handed_off = false;
  libcamera::Request *dropped    = nullptr;

  try {

    if (request->status() == libcamera::Request::RequestComplete) {

      assert(request->buffers().size() == 1);

      const libcamera::FrameBuffer *  buffer   = request->findBuffer(stream_);
      const libcamera::FrameMetadata &metadata = buffer->metadata();

      // Only hand off for the expensive work when the format is raw AND someone is listening.
      if (is_raw_ && image_pub_.getNumSubscribers() > 0) {

        std_msgs::Header hdr;
        hdr.stamp = ros::Time().fromNSec(metadata.timestamp);
        if (_use_ros_time_) {
          if (!start_time_offset_obtained_) {
            start_time_offset_          = ros::Time::now() - hdr.stamp;
            start_time_offset_obtained_ = true;
          }
          hdr.stamp += start_time_offset_;
        }
        hdr.frame_id = frame_id_;

        const buffer_info_t &binfo = buffer_info_.at(buffer);

        // hand the RAW buffer to the worker (latest-frame-wins). The copy/narrow + publish run
        // there, off this shared camera thread. fd < 0 tells the worker to skip the cache-sync.
        {
          std::scoped_lock pub_lock(publish_mutex_);
          if (pending_.request) {
            dropped = pending_.request;  // worker hasn't taken the previous frame -> we drop it
          }
          pending_.request = request;
          pending_.data    = static_cast<const uint8_t *>(binfo.data);
          pending_.size    = binfo.size;
          pending_.fd      = dmabuf_sync_ ? binfo.fd : -1;
          pending_.hdr     = hdr;
        }
        publish_cv_.notify_one();
        handed_off = true;

      } else if (!is_raw_) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: unsupported pixel format: " << stream_->configuration().pixelFormat.toString());
      }

    } else if (request->status() == libcamera::Request::RequestCancelled) {
      // Usually a PiSP frontend timeout (CSI/ISP bandwidth) or shutdown. We still re-queue
      // below so the camera can recover if it was transient.
      ROS_WARN_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: request cancelled (camera may have stalled): " << request->toString());
    }

  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: requestComplete dropped a frame: " << e.what());
  }

  // A superseded (dropped) frame's request is re-queued here -- the worker will never see it.
  if (dropped) {
    dropped->reuse(libcamera::Request::ReuseBuffers);
    if (camera_->queueRequest(dropped) < 0) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: queueRequest (dropped frame) failed");
    }
  }

  // Re-queue THIS request unless the worker now owns it (it re-queues after copying).
  if (!handed_off) {
    request->reuse(libcamera::Request::ReuseBuffers);
    if (camera_->queueRequest(request) < 0) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: queueRequest failed -- camera is no longer accepting buffers (stalled/stopped)");
    }
  }
}

//}

/* LibcameraRosDriver::publishLoop() //{ */

// Drains the single-slot handoff and runs the heavy per-frame work (copy + mono8 narrow +
// serialize + publish) off the shared camera thread. Re-queues the request as soon as the
// buffer has been copied, so capture continues while we serialize/publish.
void LibcameraRosDriver::publishLoop() {

  while (true) {

    PendingFrame f;
    {
      std::unique_lock<std::mutex> lock(publish_mutex_);
      publish_cv_.wait(lock, [this] { return pending_.request || publish_stop_; });

      if (publish_stop_) {
        return;  // shutting down: any pending frame is dropped; camera stop reclaims its buffer
      }

      f        = pending_;
      pending_ = PendingFrame();  // reset slot
    }

    // the expensive copy + narrow, now off the camera thread
    bool               built = false;
    sensor_msgs::Image img;
    try {
      img = mono8_ ? detail::fillImageMsgMono8(f.hdr, img_width_, img_height_, src_stride_, f.data, f.fd, mono8_shift_)
                   : detail::fillImageMsg(f.hdr, img_width_, img_height_, img_step_, encoding_, f.data, f.size, f.fd);
      built = true;
    }
    catch (const std::exception &e) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: frame build failed: " << e.what());
    }

    // we are done reading the buffer -> re-queue the request so the camera keeps capturing
    {
      std::scoped_lock lock(request_lock_);
      f.request->reuse(libcamera::Request::ReuseBuffers);
      if (camera_->queueRequest(f.request) < 0) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: queueRequest (worker) failed");
      }
    }

    if (!built) {
      continue;  // build failed; request already re-queued above
    }

    sensor_msgs::CameraInfo info = cinfo_msg_;
    info.header                  = f.hdr;

    // guard the publish: a throw here would otherwise terminate the whole process
    try {
      image_pub_.publish(img, info);
    }
    catch (const std::exception &e) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[LibcameraRosDriver]: publish failed: " << e.what());
    }
  }
}

//}

}  // namespace libcamera_ros_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(libcamera_ros_driver::LibcameraRosDriver, nodelet::Nodelet);
