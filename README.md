# Semi-Global Matching on GPU for ROS2

`sgm_gpu` is a ROS2 package based on [Semi-Global Matching on the GPU by D. Hernandez-Juarez](https://github.com/dhernandez0/sgm) .

It contains library, component and node to estimate disparity image from stereo images using NVIDIA GPU.

![result of ros2 launch sgm_gpu_test.py](launch_sgm_gpu_test.jpg)

## Build

Prerequisite: ROS2 Foxy and CUDA in Ubuntu 20.04

1. [Create a workspace for colcon](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)
2. Install dependencies (See package.xml)
3. Put sources in the workspace

   ```bash
   cd <the workspace>
   git clone https://github.com/fujimo-t/ros2_sgm_gpu.git src/sgm_gpu
   ```

4. Build

   ```
   colcon build
   ```

## Test

```bash
ros2 launch sgm_gpu sgm_gpu_test.py
```

Then viewers for input stereo images and output disparity are opened like above screenshot.

## Node: sgm_gpu_node

Estimate and publish disparity image from stereo image.

### Published topic

* `~/disparity` (stereo_msgs/DisparityImage)

### Subscribed topics

Remap them to topics from stereo camera.

* `left_image` (sensor_msgs/Image)
* `right_image` (sensor_msgs/Image)
* `left_camera_info` (sensor_msgs/CameraInfo)
* `right_camera_info` (sensor_msgs/CameraInfo)

### Parameter

* `image_transport` (string, default: "raw")

  See [image_transport's documentation](https://wiki.ros.org/image_transport).

## Component: sgm_gpu::SgmGpuNode

Component version of the node.

