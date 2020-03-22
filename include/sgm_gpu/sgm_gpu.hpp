/***********************************************************************
  Copyright (C) 2020 Hironori Fujimoto

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
#ifndef SGM_GPU__SGM_GPU_HPP_
#define SGM_GPU__SGM_GPU_HPP_

#include "sgm_gpu/configuration.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <opencv2/opencv.hpp>

namespace sgm_gpu
{

class SgmGpu
{
private:
  rclcpp::Logger private_logger_;
  /**
   * @brief Parameter used in SGM algorithm
   *
   * See SGM paper.
   */
  const uint8_t P1_ = 6;
  /**
   * @brief Parameter used in SGM algorithm
   *
   * See SGM paper.
   */
  const uint8_t P2_ = 96;

  // Memory for disparity computation
  // d_: for GPU device
  uint8_t *d_im0_;
  uint8_t *d_im1_;
  uint32_t *d_transform0_;
  uint32_t *d_transform1_;
  uint8_t *d_cost_;
  uint8_t *d_disparity_;
  uint8_t *d_disparity_filtered_uchar_;
  uint8_t *d_disparity_right_;
  uint8_t *d_L0_;
  uint8_t *d_L1_;
  uint8_t *d_L2_;
  uint8_t *d_L3_;
  uint8_t *d_L4_;
  uint8_t *d_L5_;
  uint8_t *d_L6_;
  uint8_t *d_L7_;
  uint16_t *d_s_;

  bool memory_allocated_;

  uint32_t cols_, rows_;

  void allocateMemory(uint32_t cols, uint32_t rows);
  void freeMemory();

  /**
   * @brief Resize images to be width and height divisible by 4 for limit of CUDA code
   */
  void resizeToDivisibleBy4(cv::Mat& left_image, cv::Mat& right_image);

  void convertToMsg(
    const cv::Mat_<unsigned char>& disparity, 
    const sensor_msgs::msg::CameraInfo& left_camera_info,
    const sensor_msgs::msg::CameraInfo& right_camera_info,
    stereo_msgs::msg::DisparityImage& disparity_msg
  );

public:
  /**
   * @brief Constructor which use namespace <parent>/libsgm_gpu for logging
   */
  SgmGpu(rclcpp::Logger& parent_logger);
  ~SgmGpu();

  bool computeDisparity(
    const sensor_msgs::msg::Image& left_image, 
    const sensor_msgs::msg::Image& right_image,
    const sensor_msgs::msg::CameraInfo& left_camera_info,
    const sensor_msgs::msg::CameraInfo& right_camera_info,
    stereo_msgs::msg::DisparityImage& disparity_msg
  );
};

} // namespace sgm_gpu

#endif // SGM_GPU__SGM_GPU_HPP_

