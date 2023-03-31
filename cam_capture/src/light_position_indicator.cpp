#include "cam_capture/light_position_indicator.hpp"

/**
 * @brief Construct a new BrightnessLevelDetector object
 *
 */
LightPositionIndicator::LightPositionIndicator() : Node("light_position_indicator")
{
    //declare and get the threshold value from the parameter server
    this->declare_parameter("threshold", 200);

    // Subscription declaration and callback binding
    pSubscription = this->create_subscription<sensor_msgs::msg::Image>("moving_camera_output", 1, std::bind(&LightPositionIndicator::imageCaptureCallback, this, _1));

    // Publisher declaraction and callback binding
    pPublisher = this->create_publisher<asdfr_interfaces::msg::Point2>("cog_input", 10);

    // Timer for callback
    pTimer = this->create_wall_timer(500ms, std::bind(&LightPositionIndicator::lightSourceCOGCallback, this));

}

/**
 * @brief Get the COG of the light source
 * 
 * @return std::array<int, 2> 
 */
std::array<float, 2> LightPositionIndicator::getLightSourceCOG()
{
    return this->mLightSourceCOG;
}

/**
 * @brief Set the COG of the light source
 * 
 * @param cog 
 */
void LightPositionIndicator::setLightSourceCOG(std::array<float, 2> cog)
{
    this->mLightSourceCOG = cog;
}

/**
 * @brief Calculate light position from image.
 *
 * @param image_msg captured image.
 * @return double
 */
void LightPositionIndicator::calculateCOG(const sensor_msgs::msg::Image &image_msg)
{
    // OpenCV image poitner
    cv_bridge::CvImagePtr cv_image;

    // Try to convert to OpenCV image
    try
    {
        cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "ERROR: cv_bridge exception: %s", e.what());
        this->setLightSourceCOG({-1, -1});
    }

    cv::Mat grayscale_image, binary_image;

    // Convert image to grayscale
    cv::cvtColor(cv_image->image, grayscale_image, cv::COLOR_BGR2GRAY);

    //read the threshold value from the parameter server
    int threshold = this->get_parameter("threshold").as_int();
    
    // Threshold the grayscaled image from value 180 to MAX_VAL with binary tresholding option
    cv::threshold(grayscale_image, binary_image, threshold, 255, cv::THRESH_BINARY);
    
    // Get moment of binary image
    cv::Moments moments = cv::moments(binary_image, true);

    // Compute center of gravity
    cv::Point2f center_of_gravity;
    center_of_gravity.x = moments.m10 / moments.m00;
    center_of_gravity.y = moments.m01 / moments.m00;

    this->setLightSourceCOG({float(center_of_gravity.x), float(center_of_gravity.y)});
}

/**
 * @brief Image capture callback funciton.
 *
 * @param image_msg
 */
void LightPositionIndicator::imageCaptureCallback(const sensor_msgs::msg::Image &image_msg)
{
    // Convert to grayscale image
    calculateCOG(image_msg);
}

/**
 * @brief Callback to publish the COG of the light source
 * 
 */
void LightPositionIndicator::lightSourceCOGCallback()
{
    // Get COG
    std::array<float, 2> COG = this->getLightSourceCOG();
    
    // Message to be published
    asdfr_interfaces::msg::Point2 arr;
    arr.x = COG[0];
    arr.y = COG[1];

    // There is no light source present
    if(COG[0] < 0 || COG[1] < 0)
    {
        // Log data
        RCLCPP_INFO(this->get_logger(), "No light source present!");

        // Publish data
        pPublisher->publish(arr);
    }
    else
    {
        // Log data
        RCLCPP_INFO(this->get_logger(), "COG: x=%.2f | y=%.2f", COG[0], COG[1]);

        // Publish value
        pPublisher->publish(arr);
    }
}


