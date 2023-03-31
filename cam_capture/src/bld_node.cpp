#include "cam_capture/brightness_level_detector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrightnessLevelDetector>());
  rclcpp::shutdown();
  return 0;
}
