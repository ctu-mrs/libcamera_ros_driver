/* includes //{ */

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <string>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libcamera/libcamera.h>
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

#include <rclcpp/rclcpp.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/node.h>

//}

namespace libcamera_ros_driver
{

  /* getCameraManager() //{ */

  // libcamera forbids more than one CameraManager per process (CameraManager ctor
  // calls LOG(Camera, Fatal) -> abort if a second is created). To run several camera
  // nodes in one component container (e.g. a true stereo pair sharing a process for
  // coherent timestamps), they must share a single manager. This returns a process-wide
  // instance, started once on first use and stopped (via ~CameraManager) when the last
  // node releases its shared_ptr.
  // ponytail: weak_ptr + mutex, the simplest correct lifetime. Upgrade only if start/stop
  // ever needs to be decoupled from node lifetime.
  static std::shared_ptr<libcamera::CameraManager> getCameraManager()
  {
    static std::mutex mtx;
    static std::weak_ptr<libcamera::CameraManager> weak;

    std::scoped_lock lock(mtx);

    if (auto cm = weak.lock())
      return cm;

    auto cm = std::make_shared<libcamera::CameraManager>();
    if (cm->start() != 0)
      throw std::runtime_error("Failed to start libcamera CameraManager");

    weak = cm;
    return cm;
  }

  //}

  /* class LibcameraRosDriver //{ */

  class LibcameraRosDriver : public mrs_lib::Node
  {
  public:
    LibcameraRosDriver(rclcpp::NodeOptions options);
    ~LibcameraRosDriver();

  private:
    rclcpp::Node::SharedPtr node_;
    void initialize();

    std::shared_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    libcamera::Stream* stream_;
    std::shared_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::mutex request_lock_;

    std::string frame_id_;

    // per-frame-constant image properties, cached at init to keep the 60 Hz callback
    // free of map lookups and a per-frame std::string heap allocation
    bool is_raw_ = false;
    std::string encoding_;
    uint32_t img_width_ = 0;
    uint32_t img_height_ = 0;
    uint32_t img_step_ = 0;
    uint32_t src_stride_ = 0;  // input row stride in bytes (for mono16->mono8 narrowing)

    // opt-in: publish MONO8 by narrowing a MONO16 frame. Halves the serialized payload for
    // consumers (e.g. VIO) that only use 8-bit greyscale. Default off keeps the raw R16 output.
    bool mono8_ = false;
    int mono8_shift_ = 8;  // PiSP unpacks raw MSB-aligned, so the top byte (>>8) is the image

    // dmabuf cache invalidate/flush around the CPU read of each frame. Necessary for correct
    // reads of NON-coherent capture buffers; pure overhead (cache ops over the whole frame) if
    // the buffers are already coherent. Exposed so it can be A/B'd on-device for the CPU it costs.
    // ponytail: default ON = always correct. Turn off only after confirming images stay intact.
    bool dmabuf_sync_ = true;

    bool _use_ros_time_ = false;

    rclcpp::Duration start_time_offset_{0, 0};
    bool start_time_offset_obtained_ = false;

    struct buffer_info_t
    {
      void* data;
      size_t size;
      int fd;  // dmabuf fd, for cache-sync around CPU reads
    };
    std::unordered_map<const libcamera::FrameBuffer*, buffer_info_t> buffer_info_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    // ponytail: cached once at init to avoid a per-frame copy + mutex inside the 60 Hz callback;
    // does not reflect runtime recalibration via the set_camera_info service. Refresh on that service if ever needed.
    sensor_msgs::msg::CameraInfo cinfo_msg_;

    image_transport::CameraPublisher image_pub_;

    // Frame offload: the libcamera requestCompleted callback runs on the CameraManager's single
    // (per-process, shared in stereo) thread, so doing the per-frame work there serializes both
    // cameras onto one core. We hand the RAW capture buffer to a per-node worker thread, which
    // does the copy + mono8 narrow + serialize + publish off the camera thread (parallel across
    // cameras) and then re-queues the request -- the buffer must stay intact until it has copied.
    // ponytail: single-slot latest-frame-wins. A queue would only add latency for a live camera;
    // a superseded frame's request is re-queued immediately so its buffer is never leaked.
    std::thread publish_thread_;
    std::mutex publish_mutex_;
    std::condition_variable publish_cv_;
    struct PendingFrame
    {
      libcamera::Request* request = nullptr;  // re-queued by the worker once the buffer is copied
      const uint8_t* data = nullptr;
      size_t size = 0;
      int fd = -1;  // dmabuf fd for cache-sync, or -1 to skip
      std_msgs::msg::Header hdr;
    };
    PendingFrame pending_;
    bool publish_stop_ = false;
    void publishLoop();

    // map parameter names to libcamera control id
    std::unordered_map<std::string, const libcamera::ControlId*> parameter_ids_;
    // parameters that are to be set for every request
    std::unordered_map<unsigned int, libcamera::ControlValue> parameters_;

    std::mutex controls_mutex_;
    std::unordered_map<unsigned int, libcamera::ControlValue> pending_controls_;
    int pending_apply_count_ = 0;

    std::unique_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
    std::string af_mode_str_;
    double lens_position_ = 0.0;

    void declareControlParameters();
    void requestComplete(libcamera::Request* request);

    bool validateControlValue(const libcamera::ControlValue& value, const libcamera::ControlId* id);
    bool updateControlParameter(const libcamera::ControlValue& value, const libcamera::ControlId* id);
    void setControlPending(const libcamera::ControlValue& value, const libcamera::ControlId* id);
    void injectPendingControls(libcamera::Request* request);
  };

  //}

  /* LibcameraRosDriver() //{ */

  LibcameraRosDriver::LibcameraRosDriver(rclcpp::NodeOptions options) : mrs_lib::Node("LibcameraRosDriver", options)
  {
    initialize();
  }

  //}

  /* initialize() //{ */

  void LibcameraRosDriver::initialize()
  {

    node_ = this_node_ptr();

    mrs_lib::ParamLoader param_loader(node_);

    std::string custom_config_path;

    param_loader.loadParam("custom_config", custom_config_path);

    if (custom_config_path != "")
    {
      param_loader.addYamlFile(custom_config_path);
    }

    param_loader.addYamlFileFromParam("config");

    std::string camera_name;
    std::string stream_role;
    std::string pixel_format;
    std::string calib_url;
    int camera_id;
    int resolution_width;
    int resolution_height;

    param_loader.loadParam("frame_id", frame_id_);
    param_loader.loadParam("calib_url", calib_url);

    param_loader.setPrefix("libcamera_ros_driver/");

    param_loader.loadParam("camera_name", camera_name);
    param_loader.loadParam("camera_id", camera_id);
    param_loader.loadParam("stream_role", stream_role);
    param_loader.loadParam("pixel_format", pixel_format);
    param_loader.loadParam("resolution/width", resolution_width);
    param_loader.loadParam("resolution/height", resolution_height);
    param_loader.loadParam("use_ros_time", _use_ros_time_);
    param_loader.loadParam("publish_mono8", mono8_, false);
    param_loader.loadParam("mono8_shift", mono8_shift_, 8);
    mono8_shift_ = std::clamp(mono8_shift_, 0, 15);  // a uint16 shift outside [0,15] is UB
    param_loader.loadParam("dmabuf_sync", dmabuf_sync_, true);

    if (!param_loader.loadedSuccessfully())
    {
      RCLCPP_ERROR(this_node().get_logger(), "Could not load all parameters!");
      rclcpp::shutdown();
      exit(1);
    }

    // start (or join) the process-wide camera manager and check for cameras
    camera_manager_ = getCameraManager();
    if (camera_manager_->cameras().empty())
    {
      RCLCPP_ERROR(this_node().get_logger(), "no cameras available");
      rclcpp::shutdown();
      exit(1);
    }

    // When camera name is specified, the camera id is automatically extracted.
    // This only works if the cameras have unique IDs.
    // When using two identical cameras, define them like this in /boot/firmware/config.txt
    //
    // dtoverlay=ov9281
    // dtoverlay=ov9281,cam0
    // dtoverlay=ov9281,cam1
    //
    // ... this will make them have a unique ID.
    // Then do `sudo rpicam-hello --list-cameras` and you will get two unique IDs, like:
    //
    // /base/axi/pcie@120000/rp1/i2c@80000/ov9281@60
    // /base/axi/pcie@120000/rp1/i2c@88000/ov9281@60

    if (!camera_name.empty())
    {

      std::vector<std::string> available_cameras;

      RCLCPP_INFO_STREAM(this_node().get_logger(), "Available cameras:");

      for (size_t i = 0; i < camera_manager_->cameras().size(); i++)
      {
        available_cameras.push_back(camera_manager_->cameras().at(i)->id());
      }

      for (size_t i = 0; i < available_cameras.size(); i++)
      {

        if (available_cameras.at(i).find(camera_name) != std::string::npos)
        {
          RCLCPP_INFO_STREAM(this_node().get_logger(), "found camera: " << camera_name << " index: " << i << " at: " << available_cameras.at(i));
          camera_id = i;
          break;
        }
      }
    }

    if (camera_id >= int(camera_manager_->cameras().size()))
    {
      RCLCPP_INFO_STREAM(this_node().get_logger(), *camera_manager_);
      RCLCPP_ERROR_STREAM(this_node().get_logger(), "camera with id " << camera_name << " does not exist");
      rclcpp::shutdown();
      exit(1);
    }

    camera_ = camera_manager_->cameras().at(camera_id);
    RCLCPP_INFO_STREAM(this_node().get_logger(), "Use camera by id: " << camera_id);

    if (!camera_)
    {
      RCLCPP_INFO_STREAM(this_node().get_logger(), *camera_manager_);
      RCLCPP_ERROR_STREAM(this_node().get_logger(), "camera with name " << camera_name << " does not exist");
      rclcpp::shutdown();
      exit(1);
    }

    if (camera_->acquire())
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to acquire camera");
      rclcpp::shutdown();
      return;
    }

    // configure camera stream
    std::unique_ptr<libcamera::CameraConfiguration> cfg = camera_->generateConfiguration({get_role(stream_role)});

    if (!cfg)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to generate configuration");
      rclcpp::shutdown();
      return;
    }

    libcamera::StreamConfiguration& scfg = cfg->at(0);

    // get common pixel formats that are supported by the camera and the node
    const libcamera::StreamFormats stream_formats = get_common_stream_formats(scfg.formats());
    const std::vector<libcamera::PixelFormat> common_fmt = stream_formats.pixelformats();

    if (common_fmt.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Camera does not provide any of the supported pixel formats");
      rclcpp::shutdown();
      return;
    }

    // Ground-truth dump: every pixel format (and its sizes) the camera offers for THIS role,
    // intersected with what the node can encode. Tells us if e.g. R8/MONO8 exists for the role
    // before we pay for any guess-and-flash cycle. Raw roles -> only sensor-native depth (R16
    // for a 10-bit sensor); processed roles (viewfinder/video) -> ISP can add R8.
    RCLCPP_INFO_STREAM(node_->get_logger(), "Supported formats for role '" << stream_role << "' (camera ∩ node):\n" << stream_formats);

    if (pixel_format.empty())
    {
      // auto select first common pixel format
      scfg.pixelFormat = common_fmt.front(); // get pixel format from provided string
      RCLCPP_INFO_STREAM(node_->get_logger(), stream_formats);
      RCLCPP_WARN_STREAM(node_->get_logger(), "No pixel format selected, using default: \"" << scfg.pixelFormat << "\"");
      RCLCPP_WARN_STREAM(node_->get_logger(), "Set parameter 'pixel_format' to silent this warning");
    } else
    {
      // get pixel format from provided string
      const libcamera::PixelFormat format_requested = libcamera::PixelFormat::fromString(pixel_format);

      if (!format_requested.isValid())
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), stream_formats);
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid pixel format: \"" << pixel_format << "\"");
        rclcpp::shutdown();
        return;
      }

      // check that the requested format is supported by camera and the node
      if (std::find(common_fmt.begin(), common_fmt.end(), format_requested) == common_fmt.end())
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), stream_formats);
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unsupported pixel format \"" << pixel_format << "\"");
        rclcpp::shutdown();
        return;
      }

      scfg.pixelFormat = format_requested;
    }

    const libcamera::Size size(resolution_width, resolution_height);

    if (size.isNull())
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), scfg);
      scfg.size = scfg.formats().sizes(scfg.pixelFormat).back();
      RCLCPP_WARN_STREAM(node_->get_logger(), "No dimensions selected, auto-selecting: \"" << scfg.size << "\"");
      RCLCPP_WARN_STREAM(node_->get_logger(), "Set parameters 'resolution/width' and 'resolution/height' to silent this warning");
    } else
    {
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

    switch (cfg->validate())
    {

    case libcamera::CameraConfiguration::Valid: {
      break;
    }

    case libcamera::CameraConfiguration::Adjusted: {
      if (selected_scfg.pixelFormat != scfg.pixelFormat)
        RCLCPP_INFO_STREAM(node_->get_logger(), stream_formats);

      if (selected_scfg.size != scfg.size)
        RCLCPP_INFO_STREAM(node_->get_logger(), scfg);

      RCLCPP_WARN_STREAM(node_->get_logger(), "Stream configuration adjusted from \"" << selected_scfg.toString() << "\" to \"" << scfg.toString() << "\"");
      break;
    }

    case libcamera::CameraConfiguration::Invalid: {
      RCLCPP_ERROR(node_->get_logger(), "Failed to valid stream configuration");
      rclcpp::shutdown();

      return;
      break;
    }
    }

    if (camera_->configure(cfg.get()) < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to configure streams");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Camera \"" << camera_->id() << "\" configured with " << scfg.toString() << " stream");
    declareControlParameters();

    int param_int;
    float param_float;
    std::string param_string;
    // bool param_bool; // Removed
    std::vector<int64_t> param_vector_int;

    param_loader.loadParam("control/exposure_time", param_int, 20);
    if (parameter_ids_.count("ExposureTime"))
      updateControlParameter(pv_to_cv(param_int, parameter_ids_["ExposureTime"]->type()), parameter_ids_["ExposureTime"]);

    param_loader.loadParam("control/fps", param_float, 20.0f);
    if (parameter_ids_.count("FrameDurationLimits"))
    {
      int64_t frame_time = 1000000 / param_float;
      updateControlParameter(pv_to_cv(std::vector<int64_t>{frame_time, frame_time}, parameter_ids_["FrameDurationLimits"]->type()),
                             parameter_ids_["FrameDurationLimits"]);
    }

    param_loader.loadParam("control/ae_constraint_mode", param_string, std::string("normal"));
    if (parameter_ids_.count("AeConstraintMode"))
      updateControlParameter(pv_to_cv(get_ae_constraint_mode(param_string), parameter_ids_["AeConstraintMode"]->type()), parameter_ids_["AeConstraintMode"]);

    param_loader.loadParam("control/brightness", param_float, 0.0f);
    if (parameter_ids_.count("Brightness"))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Brightness"]->type()), parameter_ids_["Brightness"]);

    param_loader.loadParam("control/sharpness", param_float, 1.0f);
    if (parameter_ids_.count("Sharpness"))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Sharpness"]->type()), parameter_ids_["Sharpness"]);

    std::string param_awb_enable_str = "true";
    param_loader.loadParam("control/awb_enable", param_awb_enable_str, std::string("true"));
    bool param_awb_enable_bool = (param_awb_enable_str == "true");
    if (parameter_ids_.count("AwbEnable"))
      updateControlParameter(pv_to_cv(param_awb_enable_bool, parameter_ids_["AwbEnable"]->type()), parameter_ids_["AwbEnable"]);

    /* updateControlParameter<std::vector<float>>(std::string("control.colour_gains"), parameter_ids_["ColourGains"]); */
    std::string param_ae_enable_str = "true";
    param_loader.loadParam("control/ae_enable", param_ae_enable_str, std::string("true"));
    bool param_ae_enable_bool = (param_ae_enable_str == "true");
    if (parameter_ids_.count("AeEnable"))
      updateControlParameter(pv_to_cv(param_ae_enable_bool, parameter_ids_["AeEnable"]->type()), parameter_ids_["AeEnable"]);

    param_loader.loadParam("control/saturation", param_float, 1.0f);
    if (parameter_ids_.count("Saturation"))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Saturation"]->type()), parameter_ids_["Saturation"]);

    param_loader.loadParam("control/contrast", param_float, 1.0f);
    if (parameter_ids_.count("Contrast"))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Contrast"]->type()), parameter_ids_["Contrast"]);

    param_loader.loadParam("control/exposure_value", param_float, 0.0f);
    if (parameter_ids_.count("ExposureValue"))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["ExposureValue"]->type()), parameter_ids_["ExposureValue"]);

    param_loader.loadParam("control/analogue_gain", param_float, 1.0f);
    if (parameter_ids_.count("AnalogueGain"))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["AnalogueGain"]->type()), parameter_ids_["AnalogueGain"]);

    param_loader.loadParam("control/awb_mode", param_string, std::string("auto"));
    if (parameter_ids_.count("AwbMode"))
      updateControlParameter(pv_to_cv(get_awb_mode(param_string), parameter_ids_["AwbMode"]->type()), parameter_ids_["AwbMode"]);

    param_loader.loadParam("control/ae_metering_mode", param_string, std::string("centre-weighted"));
    if (parameter_ids_.count("AeMeteringMode"))
      updateControlParameter(pv_to_cv(get_ae_metering_mode(param_string), parameter_ids_["AeMeteringMode"]->type()), parameter_ids_["AeMeteringMode"]);

    // scaler_crop is optional: empty default means "no crop", so a missing param is not an error
    param_vector_int.clear();
    param_loader.loadParam("control/scaler_crop", param_vector_int, std::vector<int64_t>{});
    if (!param_vector_int.empty() && parameter_ids_.count("ScalerCrop"))
      updateControlParameter(pv_to_cv(param_vector_int, parameter_ids_["ScalerCrop"]->type()), parameter_ids_["ScalerCrop"]);

    param_loader.loadParam("control/ae_exposure_mode", param_string, std::string("normal"));
    if (parameter_ids_.count("AeExposureMode"))
      updateControlParameter(pv_to_cv(get_ae_exposure_mode(param_string), parameter_ids_["AeExposureMode"]->type()), parameter_ids_["AeExposureMode"]);

    param_loader.loadParam("control/af_range", param_string, std::string("normal"));
    if (parameter_ids_.count("AfRange"))
      updateControlParameter(pv_to_cv(get_af_range(param_string), parameter_ids_["AfRange"]->type()), parameter_ids_["AfRange"]);

    param_loader.loadParam("control/af_speed", param_string, std::string("normal"));
    if (parameter_ids_.count("AfSpeed"))
      updateControlParameter(pv_to_cv(get_af_speed(param_string), parameter_ids_["AfSpeed"]->type()), parameter_ids_["AfSpeed"]);

    /* dynamic focus parameters //{ */

    dynparam_mgr_ = std::make_unique<mrs_lib::DynparamMgr>(node_, controls_mutex_);

    std::string config_path;
    if (node_->has_parameter("config"))
      config_path = node_->get_parameter("config").as_string();

    mrs_lib::ParamProvider& focus_pp = dynparam_mgr_->get_param_provider();
    focus_pp.setPrefix("libcamera_ros_driver/control/");
    if (!custom_config_path.empty())
      focus_pp.addYamlFile(custom_config_path);
    if (!config_path.empty())
      focus_pp.addYamlFile(config_path);

    mrs_lib::DynparamMgr::update_cbk_t<std::string> af_mode_cbk = [this](const std::string& mode) {
      if (!parameter_ids_.count("AfMode"))
        return;
      try
      {
        setControlPending(pv_to_cv(get_af_mode(mode), parameter_ids_["AfMode"]->type()), parameter_ids_["AfMode"]);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "af_mode update rejected: " << e.what());
      }
    };

    mrs_lib::DynparamMgr::update_cbk_t<double> lens_position_cbk = [this](const double& position) {
      if (!parameter_ids_.count("LensPosition"))
        return;
      setControlPending(pv_to_cv(position, parameter_ids_["LensPosition"]->type()), parameter_ids_["LensPosition"]);
    };

    dynparam_mgr_->register_param<std::string>("af_mode", &af_mode_str_, std::string("continuous"), af_mode_cbk);

    mrs_lib::DynparamMgr::range_t<double> lens_range{0.0, 20.0};
    if (parameter_ids_.count("LensPosition"))
    {
      const libcamera::ControlInfo& lens_ci = camera_->controls().at(parameter_ids_["LensPosition"]);
      if (!lens_ci.min().isNone())
        lens_range.minimum = lens_ci.min().get<float>();
      if (!lens_ci.max().isNone())
        lens_range.maximum = lens_ci.max().get<float>();
    }
    dynparam_mgr_->register_param<double>("lens_position", &lens_position_, 0.0, lens_range, lens_position_cbk);

    if (!dynparam_mgr_->loaded_successfully())
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not load all focus parameters!");
      rclcpp::shutdown();
      exit(1);
    }

    if (parameter_ids_.count("AfMode"))
      updateControlParameter(pv_to_cv(get_af_mode(af_mode_str_), parameter_ids_["AfMode"]->type()), parameter_ids_["AfMode"]);

    if (parameter_ids_.count("LensPosition"))
      updateControlParameter(pv_to_cv(lens_position_, parameter_ids_["LensPosition"]->type()), parameter_ids_["LensPosition"]);

    //}

    // cache per-frame-constant image properties (format/size are fixed after configure)
    is_raw_ = (format_type(scfg.pixelFormat) == FormatType::RAW);
    encoding_ = get_ros_encoding(scfg.pixelFormat);
    img_width_ = scfg.size.width;
    img_height_ = scfg.size.height;
    src_stride_ = scfg.stride;
    img_step_ = scfg.stride;

    // mono8 narrowing only applies to a 16-bit mono source; otherwise fall back to passthrough
    if (mono8_ && encoding_ == "mono16")
    {
      encoding_ = "mono8";
      img_step_ = img_width_;  // 1 byte/pixel, tightly packed
      RCLCPP_INFO_STREAM(node_->get_logger(), "publish_mono8: narrowing MONO16 -> MONO8 (shift " << mono8_shift_ << ")");
    } else if (mono8_)
    {
      mono8_ = false;
      RCLCPP_WARN_STREAM(node_->get_logger(), "publish_mono8 requested but source encoding is '" << encoding_ << "', not mono16; publishing unchanged");
    }

    if (!dmabuf_sync_)
      RCLCPP_WARN(node_->get_logger(), "dmabuf_sync disabled: skipping cache invalidate around frame reads. "
                                       "Verify images are intact -- non-coherent buffers can read stale/torn data.");

    // allocate stream buffers and create one request per buffer
    stream_ = scfg.stream();

    allocator_ = std::make_shared<libcamera::FrameBufferAllocator>(camera_);
    allocator_->allocate(stream_);

    for (const std::unique_ptr<libcamera::FrameBuffer>& buffer : allocator_->buffers(stream_))
    {
      std::unique_ptr<libcamera::Request> request = camera_->createRequest();

      if (!request)
      {
        RCLCPP_ERROR(node_->get_logger(), "Can't create request");
        rclcpp::shutdown();
        return;
      }

      // multiple planes of the same buffer use the same file descriptor
      size_t buffer_length = 0;
      int fd = -1;

      for (const libcamera::FrameBuffer::Plane& plane : buffer->planes())
      {
        if (plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset)
        {
          RCLCPP_ERROR(node_->get_logger(), "Invalid offset");
          rclcpp::shutdown();
          return;
        }

        buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);
        if (!plane.fd.isValid())
        {
          RCLCPP_ERROR(node_->get_logger(), "File descriptor is not valid");
          rclcpp::shutdown();
          return;
        }

        if (fd == -1)
        {
          fd = plane.fd.get();
        } else if (fd != plane.fd.get())
        {
          RCLCPP_ERROR(node_->get_logger(), "Plane file descriptors differ");
          rclcpp::shutdown();
          return;
        }
      }

      // memory-map the frame buffer planes
      void* data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);

      if (data == MAP_FAILED)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Mmap failed: " << std::string(std::strerror(errno)));
        rclcpp::shutdown();
        return;
      }

      buffer_info_[buffer.get()] = {data, buffer_length, fd};
      if (request->addBuffer(stream_, buffer.get()) < 0)
      {
        RCLCPP_ERROR(node_->get_logger(), "Can't set buffer for request");
        rclcpp::shutdown();
        return;
      }

      {
        std::scoped_lock controls_lock(controls_mutex_);
        for (const auto& [id, value] : parameters_)
          request->controls().set(id, value);
      }

      requests_.push_back(std::move(request));
    }

    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_->get_node_base_interface(), node_->get_node_services_interface(),
                                                                      node_->get_node_logging_interface(), camera_name, calib_url);
    cinfo_msg_ = cinfo_->getCameraInfo();

    /* initialize publishers //{ */

    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("~/image_raw", 1);

    //}

    // register callback
    camera_->requestCompleted.connect(this, &LibcameraRosDriver::requestComplete);

    // start camera and queue all requests
    if (camera_->start())
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to start camera");
      rclcpp::shutdown();
      return;
    }

    // start the publish worker before frames begin arriving
    publish_thread_ = std::thread(&LibcameraRosDriver::publishLoop, this);

    for (std::unique_ptr<libcamera::Request>& request : requests_)
      camera_->queueRequest(request.get());

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  //}

  /* ~LibcameraRosDriver() //{ */

  LibcameraRosDriver::~LibcameraRosDriver()
  {
    camera_->requestCompleted.disconnect();  // no more frames handed off after this

    // Stop the worker BEFORE the camera: the worker calls queueRequest, so it must be done
    // before we stop the camera. Any frame still pending is dropped; camera stop reclaims it.
    {
      std::scoped_lock lock(publish_mutex_);
      publish_stop_ = true;
    }
    publish_cv_.notify_one();
    if (publish_thread_.joinable())
      publish_thread_.join();

    {
      std::scoped_lock lock(request_lock_);

      if (camera_->stop())
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to stop camera");
      }
    }

    camera_->release();
    // Do not stop the manager here: it is shared. Dropping our shared_ptr (member
    // destruction) stops it only once the last camera node is gone.

    for (const auto& e : buffer_info_)
    {
      if (munmap(e.second.data, e.second.size) == -1)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Munmap failed: " << std::strerror(errno));
      }
    }
  }

  //}

  /* declareControlParameters() //{ */

  void LibcameraRosDriver::declareControlParameters()
  {
    RCLCPP_INFO(node_->get_logger(), "Available control parameters:");

    for (const auto& [id, info] : camera_->controls())
    {
      // std::size_t extent;
      try
      {
        get_extent(id);
      }
      catch (const std::runtime_error& e)
      {
        // ignore
        RCLCPP_INFO_STREAM(node_->get_logger(), id->name() << " : Not handled by the current version of the libcamera SDK");
        continue;
      }

      // store control id with name
      parameter_ids_[id->name()] = id;

      if (info.min().numElements() != info.max().numElements())
      {
        RCLCPP_ERROR(node_->get_logger(), "Minimum and maximum parameter array sizes do not match");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO_STREAM(node_->get_logger(), id->name()
                                                  << " : " << info.toString() << (info.def().isNone() ? "" : " (default: {" + info.def().toString() + "})"));
    }
  }

  //}

  /* validateControlValue() //{ */

  bool LibcameraRosDriver::validateControlValue(const libcamera::ControlValue& value, const libcamera::ControlId* id)
  {

    if (value.isNone())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), id->name() << " : Parameter type not defined");
      return false;
    }

    // verify parameter type and dimension against default
    const libcamera::ControlInfo& ci = camera_->controls().at(id);

    if (value.type() != id->type())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), id->name() << " : Parameter types mismatch, expected '" << std::to_string(id->type()).c_str() << "', got '"
                                                          << std::to_string(value.type()).c_str() << "'");
      return false;
    }

    const std::size_t extent = get_extent(id);
    if ((value.isArray() && (extent > 0)) && value.numElements() != extent)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), id->name() << " : Parameter dimensions mismatch, expected " << std::to_string(extent).c_str() << ", got "
                                                          << std::to_string(value.numElements()).c_str());
      return false;
    }

    // check bounds and return error
    // it seems that for exposition the 0 is used for maximum value, which means infinity
    // therefore, we are checking if max > min. If yes, check  if the value is lower than max.
    if (value < ci.min() || (ci.max() > ci.min() ? value > ci.max() : false))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), id->name() << " : Parameter value " << value.toString() << " outside of range: " << ci.toString().c_str());
      return false;
    }

    return true;
  }

  //}

  /* updateControlParameter() //{ */

  bool LibcameraRosDriver::updateControlParameter(const libcamera::ControlValue& value, const libcamera::ControlId* id)
  {
    if (!validateControlValue(value, id))
      return false;

    parameters_[id->id()] = value;
    return true;
  }

  //}

  /* setControlPending() //{ */

  void LibcameraRosDriver::setControlPending(const libcamera::ControlValue& value, const libcamera::ControlId* id)
  {
    if (!validateControlValue(value, id))
      return;

    parameters_[id->id()] = value;
    pending_controls_[id->id()] = value;
    pending_apply_count_ = static_cast<int>(requests_.size());
  }

  //}

  /* injectPendingControls() //{ */

  void LibcameraRosDriver::injectPendingControls(libcamera::Request* request)
  {
    std::scoped_lock controls_lock(controls_mutex_);

    if (pending_apply_count_ <= 0)
      return;

    for (const auto& [id, value] : pending_controls_)
      request->controls().set(id, value);

    if (--pending_apply_count_ <= 0)
      pending_controls_.clear();
  }

  //}

  /* requestComplete() //{ */

  void LibcameraRosDriver::requestComplete(libcamera::Request* request)
  {
    std::scoped_lock lock(request_lock_);

    // If we hand the buffer to the worker, the worker re-queues the request (after it has copied
    // out of the buffer); we must NOT re-queue here or the buffer would be reused mid-read. Every
    // other path re-queues below, so a request can never leave without going back to the pool --
    // a leaked request drains the finite pool and silently stops the camera forever.
    bool handed_off = false;
    libcamera::Request* dropped = nullptr;

    try
    {
      if (request->status() == libcamera::Request::RequestComplete)
      {
        assert(request->buffers().size() == 1);

        const libcamera::FrameBuffer* buffer = request->findBuffer(stream_);
        const libcamera::FrameMetadata& metadata = buffer->metadata();

        // Only hand off for the expensive work when the format is raw AND someone is listening.
        if (is_raw_ && image_pub_.getNumSubscribers() > 0)
        {
          std_msgs::msg::Header hdr;
          hdr.stamp = rclcpp::Time(metadata.timestamp);
          if (_use_ros_time_)
          {
            if (!start_time_offset_obtained_)
            {
              start_time_offset_ = node_->now() - hdr.stamp;
              start_time_offset_obtained_ = true;
            }
            rclcpp::Time ts(hdr.stamp);
            ts += start_time_offset_;
            hdr.stamp = ts;
          }
          hdr.frame_id = frame_id_;

          const buffer_info_t& binfo = buffer_info_.at(buffer);

          // hand the RAW buffer to the worker (latest-frame-wins). The copy/narrow + publish run
          // there, off this shared camera thread. fd < 0 tells the worker to skip the cache-sync.
          {
            std::scoped_lock pub_lock(publish_mutex_);
            if (pending_.request)
              dropped = pending_.request;  // worker hasn't taken the previous frame -> we drop it
            pending_.request = request;
            pending_.data = static_cast<const uint8_t*>(binfo.data);
            pending_.size = binfo.size;
            pending_.fd = dmabuf_sync_ ? binfo.fd : -1;
            pending_.hdr = hdr;
          }
          publish_cv_.notify_one();
          handed_off = true;
        } else if (!is_raw_)
        {
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                       "Unsupported pixel format: " << stream_->configuration().pixelFormat.toString());
        }
      } else if (request->status() == libcamera::Request::RequestCancelled)
      {
        // Usually a PiSP frontend timeout (CSI/ISP bandwidth) or shutdown. We still re-queue
        // below so the camera can recover if it was transient.
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                    "Request cancelled (camera may have stalled): " << request->toString());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "requestComplete dropped a frame: " << e.what());
    }

    // A superseded (dropped) frame's request is re-queued here -- the worker will never see it.
    if (dropped)
    {
      dropped->reuse(libcamera::Request::ReuseBuffers);
      injectPendingControls(dropped);
      if (camera_->queueRequest(dropped) < 0)
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "queueRequest (dropped frame) failed");
    }

    // Re-queue THIS request unless the worker now owns it (it re-queues after copying).
    if (!handed_off)
    {
      request->reuse(libcamera::Request::ReuseBuffers);
      injectPendingControls(request);
      if (camera_->queueRequest(request) < 0)
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "queueRequest failed -- camera is no longer accepting buffers (stalled/stopped)");
    }
  }

  //}

  /* publishLoop() //{ */

  // Drains the single-slot handoff and runs the heavy per-frame work (copy + mono8 narrow +
  // serialize + publish) off the shared camera thread. Re-queues the request as soon as the
  // buffer has been copied, so capture continues while we serialize/publish.
  void LibcameraRosDriver::publishLoop()
  {
    while (true)
    {
      PendingFrame f;
      {
        std::unique_lock<std::mutex> lock(publish_mutex_);
        publish_cv_.wait(lock, [this] { return pending_.request || publish_stop_; });

        if (publish_stop_)
          return;  // shutting down: any pending frame is dropped; camera stop reclaims its buffer

        f = pending_;
        pending_ = PendingFrame();  // reset slot (parens: Header's default ctor is explicit)
      }

      // the expensive copy + narrow, now off the camera thread
      std::unique_ptr<sensor_msgs::msg::Image> img;
      try
      {
        img = mono8_ ? detail::fillImageMsgMono8(f.hdr, img_width_, img_height_, src_stride_, f.data, f.fd, mono8_shift_)
                     : detail::fillImageMsg(f.hdr, img_width_, img_height_, img_step_, encoding_, f.data, f.size, f.fd);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "frame build failed: " << e.what());
      }

      // we are done reading the buffer -> re-queue the request so the camera keeps capturing
      {
        std::scoped_lock lock(request_lock_);
        f.request->reuse(libcamera::Request::ReuseBuffers);
        injectPendingControls(f.request);
        if (camera_->queueRequest(f.request) < 0)
          RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "queueRequest (worker) failed");
      }

      if (!img)
        continue;  // build failed; request already re-queued above

      auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_msg_);
      info->header = f.hdr;

      // guard the publish: a throw here would otherwise terminate the whole process
      try
      {
        image_pub_.publish(std::move(img), std::move(info));
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "publish failed: " << e.what());
      }
    }
  }

  //}

} // namespace libcamera_ros_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(libcamera_ros_driver::LibcameraRosDriver);
