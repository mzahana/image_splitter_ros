#include "rclcpp/rclcpp.hpp"
#include "stereo_image_splitter.cpp" // Include the splitter implementation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoImageSplitter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
