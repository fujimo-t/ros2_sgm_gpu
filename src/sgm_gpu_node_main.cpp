#include "sgm_gpu_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sgm_gpu::SgmGpuNode>());
  rclcpp::shutdown();
  
  return 0;
}
