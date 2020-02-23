#ifndef SGM_GPU__SGM_GPU_COMPONENT_HPP_
#define SGM_GPU__SGM_GPU_COMPONENT_HPP_

#include <memory>

#include "image_transport/subscriber_filter.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

namespace sgm_gpu
{

class SgmGpuComponent : public rclcpp::Node
{
public:
  explicit SgmGpuComponent(const rclcpp::NodeOptions& options);
  
private:
  rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_pub_;
  
  image_transport::SubscriberFilter left_img_sub_;
  image_transport::SubscriberFilter right_img_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_caminfo_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_caminfo_sub_;
  
  using StereoSynchronizer = message_filters::TimeSynchronizer
  <
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::CameraInfo
  >;
  std::shared_ptr<StereoSynchronizer> stereo_synch_;
  
  // Parameters for Semi-Global Matching
  uint8_t sgm_p1_;
  uint8_t sgm_p2_;
  
  void stereo_callback
  (
    const sensor_msgs::msg::ImageConstSharedPtr& left_image,
    const sensor_msgs::msg::ImageConstSharedPtr& right_image,
    const sensor_msgs::msg::CameraInfoConstSharedPtr& left_caminfo,
    const sensor_msgs::msg::CameraInfoConstSharedPtr& right_caminfo
  );
}

} // namespace sgm_gpu

#endif SGM_GPU__SGM_GPU_COMPONENT_HPP_ // SGM_GPU__SGM_GPU_COMPONENT_HPP_
