# Semi-Global Matching on GPU for ROS2

Work in progress

## Build

```bash
mkdir -p colcon_ws/src
cd colcon_ws
git clone https://github.com/fujimo-t/ros2_sgm_gpu.git src/sgm_gpu
colcon build
```

## SgmGpuComponent

### Published topic

* `disparity_image` (stereo_msgs/DisparityImage)

### Subscribed topics

* `left_image` (sensor_msgs/Image)
* `right_image` (sensor_msgs/Image)
* `left_camera_info` (sensor_msgs/CameraInfo)
* `right_camera_info` (sensor_msgs/CameraInfo)

### Parameter

* `image_transport` (string, default: "raw")
