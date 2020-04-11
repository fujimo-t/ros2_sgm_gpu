# Semi-Global Matching on GPU for ROS2

Work in progress

## Build

Prerequisite: ROS2 Eloquent in Ubuntu 18.04

1. [Create a workspace for colcon](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) if you don't have
2. [Install rosdep](http://wiki.ros.org/rosdep#Installing_rosdep)
3. Put sources in the workspace

  ```bash
  cd <the workspace>
  git clone https://github.com/fujimo-t/ros2_sgm_gpu.git src/sgm_gpu
  ```

4. Install dependencies by rosdep

  ```bash
  # If you have not installed CUDA
  sudo rosdep install -i --from-path src -y
  # If you have installed CUDA
  sudo rosdep install -i --from-path src -y --skip-keys="nvidia-cuda-dev nvidia-cuda"
  ```

5. Build
  ```
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
