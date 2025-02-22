**!!! This repository constains half-baked code that may not be well-maintained and not suitable for practical purposes. !!!**  
**!!! We don't have resource to maintain this extension library. If you find an issue, please fix it by yourself and open a PR to share the solution. !!!**

# glim_ext

glim_ext is a set of extension modules for [GLIM](https://github.com/koide3/glim), 3D LiDAR mapping framework. With this package, we aim to provide reference implementations that demonstrate how GLIM can be extended through the global callback slot mechanism. 

[![EXT](https://github.com/koide3/glim_ext/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim_ext/actions/workflows/build.yml)

## Disclaimer

Each module in glim_ext uses several external libraries that employ different licensing conditions. You must carefully check and follow their licenses.

## Usage

To enable an extension module, add the so filename to `extension_modules` in `glim/config/config_ros.json`. E.g., 

```
"extension_modules": [
    "libmemory_monitor.so",
    "libstandard_viewer.so",
    "librviz_viewer.so",
    "libimu_validator.so"   // Added
  ],
```


## Config path addressing

Extension modules first try to find the corresponding config path from `config.json` in the main GLIM package. It then fallback to `config_ext.json` in the `glim_ext` package if it failed to find a config path in `config.json`.

Example (`libflat_earther.so`):

1. Tries to find `config_flat_earther` in `glim/config/config.json`. The config path may be changed by `config_path` ROS param (see [here](https://koide3.github.io/glim/quickstart.html#configuration-files)).
2. If not found, find `config_flat_earther` in `glim_ext/config/config_ext.json`.


## Example Modules

### Callback demo
- This modules subscribes to all available callbacks to demonstrate how mapping states can be retrieved

## Odometry estimation Modules

### IMU validator
- Validator for LiDAR-IMU transformation configurations.

### Velocity supressor
- Regulating the velocity range.

### ORB_SLAM odometry (Not Maintained)
- Loosely coupled visual odometry constraints based on ORB_SLAM3
- Dependency: [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (GPL-3.0)

## Global Optimization Modules

### Flat earther
- Forcing the height of proximiate submaps be the same.
- This is useful in situations only a single floor exists and the height of the sensor from the floor is mostly unchanged.

### GNSS constraints (ROS2 only)
- GNSS-based constraints for global optimization

### ScanContext Loop Detector
- Explicit loop detection based on ScanContext
- Dependency: [ScanContext](https://github.com/irapkaist/scancontext) (CC BY-NC-SA 4.0)

### DBoW Loop Detector (Not Maintained)
- Explicit loop detection based on DBoW3
- Dependency: [DBoW3](https://github.com/rmsalinas/DBow3) ([LICENSE](https://github.com/rmsalinas/DBow3/blob/master/LICENSE.txt))

## Installation

### ROS1

**!!! We do not and will not support ROS1. While some modules would still work on ROS1, no tests are conducted. !!!**

```bash
cd ~/catkin_ws/src
git clone https://github.com/koide3/glim_ext

cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### ROS2

```bash
cd ~/ros2_ws/src
git clone https://github.com/koide3/glim_ext

cd ..
colcon build
```
