#include "sgm_gpu/sgm_gpu_component.hpp"

#include "disparity_method.h"

#include <functional>

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/stereo_camera_model.h"

namespace sgm_gpu
{

SgmGpu::SgmGpu(const rclcpp::NodeOptions& options) : Node("sgm_gpu", options)
{
  disparity_pub_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1);

  std::string img_transport_type = declare_parameter("image_transport", "raw");
  sgm_p1_ = declare_parameter("p1", 6);
  sgm_p2_ = declare_parameter("p2", 96);
  check_consistency_ = declare_parameter("check_consistency", true);
  
  left_img_sub_.subscribe(this, "left_image", img_transport_type);
  right_img_sub_.subscribe(this, "right_image", img_transport_type);
  left_caminfo_sub_.subscribe(this, "left_camera_info");
  right_caminfo_sub_.subscribe(this, "right_camera_info");
  
  stereo_synch_.reset(new StereoSynchronizer(left_img_sub_, right_img_sub_, left_caminfo_sub_, right_caminfo_sub_, 5));
  using namespace std::placeholders;
  stereo_synch_->registerCallback(std::bind(&SgmGpu::stereo_callback, this, _1, _2, _3, _4));
}

void SgmGpu::stereo_callback
(
  const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
  const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info
)
{
  if (disparity_pub_->get_subscription_count() == 0)
    return;
  
  cv_bridge::CvImagePtr left_cv = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImagePtr right_cv = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::MONO8);
  
  if (left_cv->image.rows != right_cv->image.rows || left_cv->image.cols != right_cv->image.cols)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), 
      "Image dimensions of left and right are not same:\n" <<
      "Left image: " << left_cv->image.cols << "x" << left_cv->image.rows << "\n" <<
      "Right image: " << right_cv->image.cols << "x" << right_cv->image.rows
    );
    return;
  }
  
  // sgm_gpu specify image size to divisible by 4
  // Resize if input images aren't
  bool need_resize = false;
  cv::Size original_size, resized_size; 
  original_size = cv::Size(left_cv->image.cols, left_cv->image.rows);
  resized_size = original_size;
  if (original_size.width % 4 != 0)
  {
    need_resize = true;
    resized_size.width = (original_size.width / 4 + 1) * 4;
  }
  if (original_size.height % 4 != 0)
  {
    need_resize = true;
    resized_size.height = (original_size.height / 4 + 1) * 4;
  }
  
  if (need_resize)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(),
      "Resize image width and height to divisible by 4 before stereo matching\n" <<
      "The size of output is restored to original at publishing\n" <<
      "Original size: " << original_size.width << "x" << original_size.height << "\n" <<
      "Resized to: " << resized_size.width << "x" << resized_size.height
    );
    cv::resize(left_cv->image, left_cv->image, resized_size, 0, 0, cv::INTER_LINEAR);
    cv::resize(right_cv->image, right_cv->image, resized_size, 0, 0, cv::INTER_LINEAR);
  }
  
  float elapsed_time_ms;
  cv::Mat disparity_8u;
  compute_disparity_method(left_cv->image, right_cv->image, &disparity_8u, &elapsed_time_ms, check_consistency_);

  RCLCPP_INFO(this->get_logger(), "Elapsed time: %f [ms]", elapsed_time_ms);

  if (need_resize)
    cv::resize(disparity_8u, disparity_8u, original_size, 0, 0, cv::INTER_AREA);
  
  cv::Mat disparity_32f;
  disparity_8u.convertTo(disparity_32f, CV_32F);

  stereo_msgs::msg::DisparityImage disparity_msg;
  disparity_msg.header = left_image->header;

  cv_bridge::CvImage disparity_converter(left_image->header, sensor_msgs::image_encodings::TYPE_32FC1, disparity_32f);
  disparity_converter.toImageMsg(disparity_msg.image);

  image_geometry::StereoCameraModel stereo_model;
  stereo_model.fromCameraInfo(left_info, right_info);
  disparity_msg.f = stereo_model.left().fx();
  disparity_msg.t = stereo_model.baseline();

  disparity_msg.min_disparity = 0.0;
  disparity_msg.max_disparity = 128.0;
  
  disparity_msg.delta_d = 1.0;

  disparity_pub_->publish(disparity_msg);
}

} // namespace sgm_gpu
