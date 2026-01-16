/* includes //{ */

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
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

#include <rclcpp/rclcpp.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/node.h>

//}

namespace libcamera_ros_driver
{

  /* class LibcameraRosDriver //{ */

  class LibcameraRosDriver : public mrs_lib::Node
  {
  public:
    LibcameraRosDriver(rclcpp::NodeOptions options);
    ~LibcameraRosDriver();

  private:
    rclcpp::Node::SharedPtr node_;
    void initialize();

    libcamera::CameraManager camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    libcamera::Stream* stream_;
    std::shared_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::mutex request_lock_;

    std::string frame_id_;

    bool _use_ros_time_ = false;

    rclcpp::Duration start_time_offset_{0, 0};
    bool start_time_offset_obtained_ = false;

    struct buffer_info_t
    {
      void* data;
      size_t size;
    };
    std::unordered_map<const libcamera::FrameBuffer*, buffer_info_t> buffer_info_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    image_transport::CameraPublisher image_pub_;
    std::mutex image_pub_mutex_;

    // map parameter names to libcamera control id
    std::unordered_map<std::string, const libcamera::ControlId*> parameter_ids_;
    // parameters that are to be set for every request
    std::unordered_map<unsigned int, libcamera::ControlValue> parameters_;

    void declareControlParameters();
    void requestComplete(libcamera::Request* request);

    bool updateControlParameter(const libcamera::ControlValue& value, const libcamera::ControlId* id);
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

    if (!param_loader.loadedSuccessfully())
    {
      RCLCPP_ERROR(this_node().get_logger(), "Could not load all parameters!");
      rclcpp::shutdown();
      exit(1);
    }

    // start camera manager and check for cameras
    camera_manager_.start();
    if (camera_manager_.cameras().empty()){
      RCLCPP_ERROR(this_node().get_logger(), "no cameras available");
      rclcpp::shutdown();
      exit(1);
    }

    if (!camera_name.empty()){

      std::vector<std::string> available_cameras;

      RCLCPP_INFO_STREAM(this_node().get_logger(), "Available cameras:");

      for(size_t i = 0; i < camera_manager_.cameras().size(); i++){
        available_cameras.push_back(camera_manager_.cameras().at(i)->id());
      }

      for(size_t i = 0; i < available_cameras.size(); i++){

        if(available_cameras.at(i).find(camera_name) != std::string::npos && int(i) == camera_id){
          RCLCPP_INFO_STREAM(this_node().get_logger(), "found camera: " << camera_name << " index: " << i << " at: " << available_cameras.at(i));
          break;
        }
      }
    }

    if(camera_id >= int(camera_manager_.cameras().size())){
      RCLCPP_INFO_STREAM(this_node().get_logger(), camera_manager_);
      RCLCPP_ERROR_STREAM(this_node().get_logger(), "camera with id " << camera_name << " does not exist");
      rclcpp::shutdown();
      exit(1);
    }

    camera_ = camera_manager_.cameras().at(camera_id);
    RCLCPP_INFO_STREAM(this_node().get_logger(), "Use camera by id: " << camera_id);

    if (!camera_) {
      RCLCPP_INFO_STREAM(this_node().get_logger(), camera_manager_);
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
    bool param_bool;
    std::vector<int64_t> param_vector_int;

    if (param_loader.loadParam("control.exposure_time", param_int))
      updateControlParameter(pv_to_cv(param_int, parameter_ids_["ExposureTime"]->type()), parameter_ids_["ExposureTime"]);

    if (param_loader.loadParam("control.fps", param_float))
    {
      int64_t frame_time = 1000000 / param_float;
      updateControlParameter(pv_to_cv(std::vector<int64_t>{frame_time, frame_time}, parameter_ids_["FrameDurationLimits"]->type()),
                             parameter_ids_["FrameDurationLimits"]);
    }

    if (param_loader.loadParam("control.ae_constraint_mode", param_string))

      updateControlParameter(pv_to_cv(get_ae_constraint_mode(param_string), parameter_ids_["AeConstraintMode"]->type()), parameter_ids_["AeConstraintMode"]);

    if (param_loader.loadParam("control.brightness", param_float))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Brightness"]->type()), parameter_ids_["Brightness"]);

    if (param_loader.loadParam("control.sharpness", param_float))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Sharpness"]->type()), parameter_ids_["Sharpness"]);

    if (param_loader.loadParam("control.awb_enable", param_bool))
    {
      if (parameter_ids_["AwbEnable"]) // if the parameter is set when not available, we would get a segmentation fault upon extracting its ->type()
        updateControlParameter(pv_to_cv(param_bool, parameter_ids_["AwbEnable"]->type()), parameter_ids_["AwbEnable"]);
      else
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Parameter AwbEnable is not available! Maybe the selected camera is grayscale");
    }

    /* updateControlParameter<std::vector<float>>(std::string("control.colour_gains"), parameter_ids_["ColourGains"]); */
    if (param_loader.loadParam("control.ae_enable", param_bool))
      updateControlParameter(pv_to_cv(param_bool, parameter_ids_["AeEnable"]->type()), parameter_ids_["AeEnable"]);

    if (param_loader.loadParam("control.saturation", param_float))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Saturation"]->type()), parameter_ids_["Saturation"]);

    if (param_loader.loadParam("control.contrast", param_float))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["Contrast"]->type()), parameter_ids_["Contrast"]);

    if (param_loader.loadParam("control.exposure_value", param_float))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["ExposureValue"]->type()), parameter_ids_["ExposureValue"]);

    if (param_loader.loadParam("control.analogue_gain", param_float))
      updateControlParameter(pv_to_cv(param_float, parameter_ids_["AnalogueGain"]->type()), parameter_ids_["AnalogueGain"]);

    if (param_loader.loadParam("control.awb_mode", param_string))
      updateControlParameter(pv_to_cv(get_awb_mode(param_string), parameter_ids_["AwbMode"]->type()), parameter_ids_["AwbMode"]);

    if (param_loader.loadParam("control.ae_metering_mode", param_string))
      updateControlParameter(pv_to_cv(get_ae_metering_mode(param_string), parameter_ids_["AeMeteringMode"]->type()), parameter_ids_["AeMeteringMode"]);

    if (param_loader.loadParam("control.scaler_crop", param_vector_int))
      updateControlParameter(pv_to_cv(param_vector_int, parameter_ids_["ScalerCrop"]->type()), parameter_ids_["ScalerCrop"]);

    if (param_loader.loadParam("control.control", param_string))
      updateControlParameter(pv_to_cv(get_ae_exposure_mode(param_string), parameter_ids_["AeExposureMode"]->type()), parameter_ids_["AeExposureMode"]);

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

      buffer_info_[buffer.get()] = {data, buffer_length};
      if (request->addBuffer(stream_, buffer.get()) < 0)
      {
        RCLCPP_ERROR(node_->get_logger(), "Can't set buffer for request");
        rclcpp::shutdown();
        return;
      }

      // set modified control parameters
      for (const auto& [id, value] : parameters_)
        request->controls().set(id, value);

      requests_.push_back(std::move(request));
    }

    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_->get_node_base_interface(), node_->get_node_services_interface(),
                                                                      node_->get_node_logging_interface(), camera_name, calib_url);

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

    for (std::unique_ptr<libcamera::Request>& request : requests_)
      camera_->queueRequest(request.get());

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  //}

  /* ~LibcameraRosDriver() //{ */

  LibcameraRosDriver::~LibcameraRosDriver()
  {
    camera_->requestCompleted.disconnect();

    {
      std::scoped_lock lock(request_lock_);

      if (camera_->stop())
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to stop camera");
      }
    }

    camera_->release();
    camera_manager_.stop();

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

  /* updateControlParameter() //{ */

  bool LibcameraRosDriver::updateControlParameter(const libcamera::ControlValue& value, const libcamera::ControlId* id)
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

    parameters_[id->id()] = value;
    return true;
  }

  //}

  /* requestComplete() //{ */

  void LibcameraRosDriver::requestComplete(libcamera::Request* request)
  {
    std::scoped_lock lock(request_lock_);

    if (request->status() == libcamera::Request::RequestComplete)
    {
      assert(request->buffers().size() == 1);

      // get the stream and buffer from the request
      const libcamera::FrameBuffer* buffer = request->findBuffer(stream_);
      const libcamera::FrameMetadata& metadata = buffer->metadata();
      size_t bytesused = 0;

      for (const libcamera::FrameMetadata::Plane& plane : metadata.planes())
      {
        bytesused += plane.bytesused;
      }

      // send image data
      std_msgs::msg::Header hdr;

      hdr.stamp = rclcpp::Time(metadata.timestamp);
      if (_use_ros_time_)
      {
        if (!start_time_offset_obtained_)
        {
          start_time_offset_ = node_->now() - hdr.stamp;
          start_time_offset_obtained_ = true;
        }

        {
          rclcpp::Time ts(hdr.stamp);
          ts += start_time_offset_;
          hdr.stamp = ts;
        }
      }

      hdr.frame_id = frame_id_;
      const libcamera::StreamConfiguration& cfg = stream_->configuration();

      sensor_msgs::msg::Image image_msg;

      if (format_type(cfg.pixelFormat) == FormatType::RAW)
      {
        // raw uncompressed image
        assert(buffer_info_[buffer].size == bytesused);
        image_msg.header = hdr;
        image_msg.width = cfg.size.width;
        image_msg.height = cfg.size.height;
        image_msg.step = cfg.stride;
        image_msg.encoding = get_ros_encoding(cfg.pixelFormat);
        image_msg.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
        image_msg.data.resize(buffer_info_[buffer].size);

        memcpy(image_msg.data.data(), buffer_info_[buffer].data, buffer_info_[buffer].size);

      } else
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unsupported pixel format: " << stream_->configuration().pixelFormat.toString());
        return;
      }

      sensor_msgs::msg::CameraInfo cinfo_msg = cinfo_->getCameraInfo();
      cinfo_msg.header = hdr;

      {
        std::scoped_lock lock(image_pub_mutex_);
        image_pub_.publish(image_msg, cinfo_msg);
      }

    } else if (request->status() == libcamera::Request::RequestCancelled)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Request '" << request->toString() << "' cancelled");
    }

    // queue the request again for the next frame
    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
  }

  //}

} // namespace libcamera_ros_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(libcamera_ros_driver::LibcameraRosDriver);
