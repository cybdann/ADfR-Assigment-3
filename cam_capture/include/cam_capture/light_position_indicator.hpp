#include <functional>
#include <memory>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "asdfr_interfaces/msg/point2.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief Class containing the LightPositionIndicator.
 *
 */
class LightPositionIndicator : public rclcpp::Node
{
public:
    // Member funciton declaration
    LightPositionIndicator();
    void calculateCOG(const sensor_msgs::msg::Image &image_msg);

    // Getter
    std::array<float, 2> getLightSourceCOG();

    // Setter
    void setLightSourceCOG(std::array<float, 2> cog);

private:
    // Node member pointers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pSubscription;

    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr pPublisher;
    rclcpp::TimerBase::SharedPtr pTimer;


    // Member variables
    std::array<float, 2> mLightSourceCOG{-1, -1}; // Initially no light source

    // Member function declration
    void imageCaptureCallback(const sensor_msgs::msg::Image &image_msg);
    void lightSourceCOGCallback();
};