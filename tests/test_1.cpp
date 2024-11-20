// object_pickup_node_test.cpp
#include "object_pickup_node.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ObjectPickupNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = std::make_shared<ObjectPickupNode>();
    }

    std::shared_ptr<ObjectPickupNode> node_;
};

TEST_F(ObjectPickupNodeTest, TestImageCallbackDetectRedObject)
{
    // Create a dummy image with a red circle
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::circle(image, cv::Point(320, 240), 50, cv::Scalar(0, 0, 255), -1);

    // Convert to ROS Image message
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    // Call the imageCallback
    node_->imageCallback(img_msg);

    // Verify the expected result (e.g., check if a navigation goal was triggered)
    // In practice, use a mock action client to verify interactions
    // ASSERT_TRUE(some_condition);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
