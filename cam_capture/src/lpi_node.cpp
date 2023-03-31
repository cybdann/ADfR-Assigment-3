#include "cam_capture/light_position_indicator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPositionIndicator>());
  rclcpp::shutdown();
  return 0;
}
