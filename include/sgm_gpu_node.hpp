#ifndef SGM_GPU__SGM_GPU_NODE_HPP_
#define SGM_GPU__SGM_GPU_NODE_HPP_

#include "sgm_gpu/sgm_gpu.hpp"

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace sgm_gpu
{

class SgmGpuNode : public rclcpp::Node
{
public:
  explicit SgmGpuNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
private:
  std::shared_ptr<SgmGpu> sgm_gpu_;
  
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
  
  void stereo_callback
  (
    const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info
  );
};

} // namespace sgm_gpu

#endif // SGM_GPU__SGM_GPU_NODE_HPP_
