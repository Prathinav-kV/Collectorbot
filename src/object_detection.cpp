// color_detection_node.cpp
#include "object_detection.hpp"

ColorDetectionNode::ColorDetectionNode() : Node("color_detection_node")
{
    // Subscriber to the image topic from the TurtleBot3 camera
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ColorDetectionNode::imageCallback, this, std::placeholders::_1));
}

void ColorDetectionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Define color range to detect (e.g., red color in HSV space)
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Scalar lower_red1(0, 120, 70);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(170, 120, 70);
    cv::Scalar upper_red2(180, 255, 255);

    // Threshold the HSV image to get only red colors
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
    cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
    cv::bitwise_or(mask1, mask2, mask);

    // Find contours of the detected red objects
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Draw contours on the original image
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::drawContours(cv_ptr->image, contours, (int)i, cv::Scalar(0, 255, 0), 2);
    }

    // Display the image with the detected objects
    cv::imshow("Detected Objects", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
