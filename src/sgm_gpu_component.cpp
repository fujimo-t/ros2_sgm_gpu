#include "sgm_gpu_component.hpp"

#include <image_geometry/stereo_camera_model.h>
#include <functional>

namespace sgm_gpu
{

SgmGpuComponent::SgmGpuComponent(const rclcpp::NodeOptions& options) 
  : Node("sgm_gpu", options)
{
  rclcpp::Logger parent_logger = this->get_logger();
  sgm_gpu_.reset(new SgmGpu(parent_logger));
  
  disparity_pub_ 
    = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1);

  std::string img_transport_type = declare_parameter("image_transport", "raw");
  
  left_img_sub_.subscribe(this, "left_image", img_transport_type);
  right_img_sub_.subscribe(this, "right_image", img_transport_type);
  left_caminfo_sub_.subscribe(this, "left_camera_info");
  right_caminfo_sub_.subscribe(this, "right_camera_info");
  
  stereo_synch_.reset(new StereoSynchronizer(
    left_img_sub_, right_img_sub_, left_caminfo_sub_, right_caminfo_sub_, 5));
  using namespace std::placeholders;
  stereo_synch_->registerCallback(
    std::bind(&SgmGpuComponent::stereo_callback, this, _1, _2, _3, _4));
}

void SgmGpuComponent::stereo_callback
(
  const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
  const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info
)
{
  if (disparity_pub_->get_subscription_count() == 0)
    return;

  stereo_msgs::msg::DisparityImage disparity_msg;
  sgm_gpu_->computeDisparity(*left_image, *right_image, *left_info, *right_info, disparity_msg);

  disparity_pub_->publish(disparity_msg);
}

} // namespace sgm_gpu

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sgm_gpu::SgmGpuComponent)

