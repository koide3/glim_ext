# glim_ext

***glim_ext*** is a set of extension modules for ***GLIM***, 3D LiDAR mapping framework. With this package, we aim to provide reference implementations that demonstrate how GLIM can be extended through the global callback slot mechanism. Note that this is proof-of-concept code for showing the extensibility of GLIM and may not be well-maintained and not suitable for practical purposes.

## Disclaimer

Each module in glim_ext uses several external libraries that employ different licensing conditions. You must carefully check and follow their licenses.


## Example Modules

### Callback demo
- This modules subscribes to all available callbacks to demonstrate how mapping states can be retrieved

## Frontend Modules

### ORB_SLAM frontend
- Loosely coupled visual frontend constraints based on ORB_SLAM3
- Dependency: [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (GPL-3.0)

## Backend Modules

### GNSS constraints [ROS1 only]
- GNSS-based constraints for global optimization

### ScanContext Loop Detector
- Explicit loop detection based on ScanContext
- Dependency: [ScanContext](https://github.com/irapkaist/scancontext) (CC BY-NC-SA 4.0)

### DBoW Loop Detector
- Explicit loop detection based on DBoW3
- Dependency: [DBoW3](https://github.com/rmsalinas/DBow3) ([LICENSE](https://github.com/rmsalinas/DBow3/blob/master/LICENSE.txt))


## Installation

### ROS1

```bash
cd ~/catkin_ws/src
git clone glim
git clone glim_ext

cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release

# optional arguments
# catkin_make -DENABLE_SCAN_CONTEXT=ON \
#             -DENABLE_DBOW=ON \
#             -DENABLE_ORBSLAM=ON
```

### ROS2

```bash
concon build
```