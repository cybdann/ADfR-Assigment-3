#include "cam_capture/brightness_level_detector.hpp"

/**
 * @brief Construct a new BrightnessLevelDetector object
 *
 */
BrightnessLevelDetector::BrightnessLevelDetector() : Node("brightness_level_detector")
{
    // Subscription declaration and callback binding
    pSubscription = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&BrightnessLevelDetector::imageCaptureCallback, this, _1));

    // Publisher declaraction and callback binding
    pPublisher = this->create_publisher<std_msgs::msg::String>("light_detector", 10);

    // Timer for callback
    pTimer = this->create_wall_timer(500ms, std::bind(&BrightnessLevelDetector::lightDetectionCallback, this));

    // Parameter name declaration and default value
    this->declare_parameter("threshold", 30.0);
}

/**
 * @brief Calculate brightness level from the image matrix.
 *
 * @param image_msg captured image.
 * @return float
 */
double BrightnessLevelDetector::calculateBrightness(const sensor_msgs::msg::Image image_msg)
{
    int size = image_msg.width * image_msg.height;
    int sum = 0;

    // Calculate average pixel brightness
    for (int i = 0; i < size; ++i)
    {
        sum += int(image_msg.data.at(i));
    }

    return double(sum / size);
}

/**
 * @brief Check if the light is ON or OFF
 *
 * @param brightness average pixel value from image matrix.
 * @return std_msgs::msg::String
 */
std_msgs::msg::String BrightnessLevelDetector::checkLight(double brightness)
{
    std_msgs::msg::String state;

    // Compare brightness to set threshold
    if (brightness < this->get_parameter("threshold").get_parameter_value().get<double>())
    {
        state.set__data("OFF");
    }
    else
    {
        state.set__data("ON");
    }

    return state;
}

/**
 * @brief Image capture callback funciton.
 *
 * @param image_msg
 */
void BrightnessLevelDetector::imageCaptureCallback(const sensor_msgs::msg::Image &image_msg)
{
    // Get brightness level
    mBrightness = calculateBrightness(image_msg);

    // Log data
    RCLCPP_INFO(this->get_logger(), "The average brightness of image is: %.2f", mBrightness);
}

/**
 * @brief Lidght detection callback function.
 *
 */
void BrightnessLevelDetector::lightDetectionCallback()
{
    // Get parameter value
    double threshold = this->get_parameter("threshold").get_parameter_value().get<double>();

    // Check if the light is ON or OFF
    mLight = checkLight(mBrightness);

    // Log data
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s' with threshold set to: %.2f", mLight.data.c_str(), threshold);

    // Publish value
    pPublisher->publish(mLight);
}
