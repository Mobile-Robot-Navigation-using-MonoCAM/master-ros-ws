# ROS Project
All ROS workspace belongs to two wheeled robot are here!

## Test Environment
- ROS: noetic
- Ubuntu: 20.04LTS
- Python: 3.8.10
- (Gazebo: 11.9.0)

# About stella_vslam_ros

[stella_vslam](https://github.com/stella-cv/stella_vslam)'s ROS package.

## Install instruction

stella_vslam_ros uses submodules. Clone it with `git clone --recursive` or download submodules with `git submodule update --init --recursive`.

## Subscribed topics

### monocular setup

- `camera/image_raw`

### stereo setup

- `camera/left/image_raw`
- `camera/right/image_raw`

### RGBD setup

- `camera/color/image_raw`
- `camera/depth/image_raw`

## Published topics

- `~/camera_pose`
- `~/pointcloud`
- `/tf`

## Parameters

- `odom_frame`
- `map_frame`
- `base_link`
- `camera_frame`
- `publish_tf`
- `publish_pointcloud`
- `transform_tolerance`
- `use_exact_time` (stereo, RGBD only)
