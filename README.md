# Libcamera ROS driver

https://github.com/ctu-mrs/libcamera_ros/tree/ros2

## Description

### Libcamera

libcamera is a new software library aimed at supporting complex camera systems directly from the Linux operating system. 
It enables us to drive the Raspberry Pi camera system directly from open-source code running on Arm processors. 
The proprietary code running on the Broadcom GPU, to which users have no access, is almost completely bypassed.

libcamera presents a C++ API to applications. 
It works at the level of configuring the camera and then allowing an application to request image frames. 
These image buffers reside in system memory and can be passed directly to still image encoders (such as JPEG) or to video encoders (such as h.264). 
Ancillary functions such as encoding images or displaying them are beyond the purview of libcamera itself.

This package does the wrapping for ROS.

Check the configuration file for available control parameters: [config.yaml](https://github.com/ctu-mrs/libcamera_ros/blob/main/config/param.yaml).

## Prequisites

The `libcamera` library is required to be installed and in the present path (must be findable by CMake). Installation instructions:

1. Enable installation from MRS PPA [README.md](https://github.com/ctu-mrs/mrs_uav_system/tree/ros2?tab=readme-ov-file#native-installation). The stable PPA version is recommended.
2. install libcamera package `sudo apt install ros-jazzy-libcamera` 

Deb packages are available for arm64 and amd64 architectures. The driver is supposed to be used on arm64 (Raspberry Pi5). 
However, installing it on amd64 makes it easier to work with during development.

It was tested with ROS Noetic, but it should also work with older/newer versions.

## Running

Basic launch file without loading parameters from config file

```bash
ros2 launch libcamera_ros basic.launch 
```

More complex launch file using the parameters from config file and namespace UAV.

```bash
ros2 launch libcamera_ros uav.launch 
```

NOTE: This launch file also accepts custom_config argument to load a custom config file, that can override the default parameters.

### Config override logic

Parameters are resolved in this order (first match wins):

```
custom_config (e.g. camera_front.yaml)  >  config/default.yaml  >  ROS params (frame_id, calib_url from the launch file)
```

`custom_config` is added before `default.yaml`, so it only needs to contain the keys that differ from the default (typically the sensor selection and per-camera tweaks); everything it omits falls through to `default.yaml`.

### Stereo / multi-camera (single container)

libcamera allows only **one `CameraManager` per process** (its constructor aborts otherwise). To run several cameras you must therefore load them into a **single component container**, which is also exactly what you want for a stereo rig, because both nodes then share one process and one clock domain, giving directly comparable timestamps.

`stereo.launch.py` does this: it builds one `component_container_isolated` holding two driver nodes. Each node loads the same `default.yaml` plus its own `custom_config` that selects a distinct sensor.

```bash
ros2 launch libcamera_ros_driver stereo.launch.py \
  left_name:=front  left_custom_config:=/abs/path/camera_front.yaml  left_calib_url:="file:///abs/path/front_calib.yaml" \
  right_name:=back  right_custom_config:=/abs/path/camera_back.yaml   right_calib_url:="file:///abs/path/back_calib.yaml"
```

Each per-camera config must select a **different** sensor, either by index:

```yaml
libcamera_ros_driver:
  camera_name: ""   # empty disables name matching
  camera_id: 0      # 0 / 1
```

or by the unique i2c path (more robust across reboots; see the `dtoverlay ...,cam0/cam1` setup in `config.txt`):

```yaml
libcamera_ros_driver:
  camera_name: "i2c@80000"   # / "i2c@88000"
```

Minimal example overrides ship as `config/camera_left.yaml` and `config/camera_right.yaml`.

**Synchronization caveat.** Sharing a container gives a shared clock domain (comparable stamps) but does **not** by itself align the sensors' exposures, without hardware triggering the two sensors free-run and their phases drift across roughly ±½ frame period. For VIO-grade stereo the sensors must be hardware-synced (FSIN/strobe wiring). To verify, use the bundled helper to watch `t_left − t_right`:

```bash
ros2 run libcamera_ros_driver check_stereo_sync.py \
  --left  /uav1/rpi_camera_front/image_raw \
  --right /uav1/rpi_camera_back/image_raw
```

A tight, drift-free constant ⇒ hardware-synced (the constant is a fixed phase offset, fine to ignore or calibrate); a value that wanders across several ms ⇒ free-running.


## Acknowledgements

The code was inspired by the driver for ROS2: https://github.com/christianrauch/camera_ros.
