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

  See [image_transport's documentation](https://wiki.ros.org/image_transport).

* `p1` (uint8, default: 6)

  Used at cost aggregation step. 
  Penalty value if difference of disparity between neighborhood pixel is small. 
  
  See paper for detail: `H. Hirschmuller, "Stereo Processing by Semiglobal Matching and Mutual Information," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 30, no. 2, pp. 328-341, Feb. 2008.`.

* `p2` (uint8, default: 96)

  Used at cost aggregation step. 
  Penalty value if difference of disparity between neighborhood pixel is large. 
  
  See the paper for detail.

* `check_consistency` (bool, default: true)

  Enable/disable left-right consistency check.
