#include "sgm_gpu_component.hpp"

#include <functional>

namespace sgm_gpu
{

SgmGpuComponent::SgmGpuComponent() : Node("sgm_gpu", options)
{
  disparity_pub_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1);

  std::string img_transport_type;
  try
  {
    if (get_parameter("image_transport", img_transport_type))
      img_transport_type = "raw";
  }
  catch (rclcpp::exceptions::ParameterTypeException& exception)
    RCLCPP_FATAL(get_logger(), "Invalid image_transport parameter type: %s", exception.what().c_str());
  
  left_img_sub_.subscribe(this, "left_image", img_transport_type);
  right_img_sub_.subscribe(this, "right_image", img_transport_type);
  left_caminfo_sub_.subscribe(this, "left_camera_info");
  right_caminfo_sub_.subscribe(this, "right_camera_info");
  // synchronizer
  stereo_synch_.reset(new StereoSynchronizer(left_img_sub_, right_img_sub_, left_caminfo_sub_, right_caminfo_sub_), 5);
  stereo_synch_->registerCallback(std::bind(&SgmGpuComponent::stereoCallback, this, _1, _2, _3, _4));
}

void SgmGpuComponent::stereoCallback
(
  const sensor_msgs::msg::ImageConstSharedPtr& left_image,
  const sensor_msgs::msg::ImageConstSharedPtr& right_image,
  const sensor_msgs::msg::CameraInfoConstSharedPtr& left_caminfo,
  const sensor_msgs::msg::CameraInfoConstSharedPtr& right_caminfo
)
{
}

} // namespace sgm_gpu
