// object_pickup_node.cpp
#include "object_pickup_node.hpp"

/**
 * @brief Constructor for the ObjectPickupNode class.
 */
ObjectPickupNode::ObjectPickupNode() : Node("object_pickup_node")
{
    // Subscriber to the image topic from the TurtleBot3 camera
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ObjectPickupNode::imageCallback, this, std::placeholders::_1));

    // Create navigation action client
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");
}

/**
 * @brief Callback function for processing incoming images to detect objects.
 * @param msg Shared pointer to the image message.
 */
void ObjectPickupNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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

    // If an object is detected, navigate towards it
    if (!contours.empty())
    {
        // For simplicity, consider the largest contour as the target object
        size_t largest_contour_index = 0;
        double largest_area = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area)
            {
                largest_area = area;
                largest_contour_index = i;
            }
        }

        // Calculate the centroid of the largest contour
        cv::Moments m = cv::moments(contours[largest_contour_index]);
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);

        // Create a target pose to navigate towards the object (simplified)
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = 1.0;  // Assume 1 meter forward
        target_pose.pose.position.y = 0.0;
        target_pose.pose.orientation.w = 1.0;

        navigateToObject(target_pose);
    }
}

/**
 * @brief Sends a navigation goal to move towards the detected object.
 * @param target_pose The target pose for the navigation.
 */
void ObjectPickupNode::navigateToObject(const geometry_msgs::msg::PoseStamped &target_pose)
{
    // Wait for the navigation action server to be available
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Navigation action server not available after waiting");
        return;
    }

    // Create a goal message for the navigation action
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = target_pose;

    // Send the goal to the action server
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const auto &)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation to object complete, preparing to pick up");
        pickUpObject();
    };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
}

/**
 * @brief Uses MoveIt to control the manipulator and pick up the detected object.
 */
void ObjectPickupNode::pickUpObject()
{
    // Use MoveIt to control the manipulator arm and pick up the object
    moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "arm");
    move_group.setNamedTarget("pick");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group.execute(my_plan);
        RCLCPP_INFO(this->get_logger(), "Object picked up successfully");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan for picking up the object");
    }
}

/**
 * @brief Main function to run the ObjectPickupNode.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectPickupNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
