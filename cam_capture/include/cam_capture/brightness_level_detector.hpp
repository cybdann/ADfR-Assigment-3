#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief Class containing the BrightnessLevelDetector.
 *
 */
class BrightnessLevelDetector : public rclcpp::Node
{
public:
    // Member funciton declaration
    BrightnessLevelDetector();
    double calculateBrightness(const sensor_msgs::msg::Image image_msg);
    std_msgs::msg::String checkLight(double brightness);

private:
    // Node member pointers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pSubscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pPublisher;
    rclcpp::TimerBase::SharedPtr pTimer;

    // Member variables
    std_msgs::msg::String mLight;
    double mBrightness;

    // Member function declration
    void imageCaptureCallback(const sensor_msgs::msg::Image &image_msg);
    void lightDetectionCallback();
};

