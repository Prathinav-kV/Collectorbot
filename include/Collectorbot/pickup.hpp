// object_pickup_node.hpp
#ifndef OBJECT_PICKUP_NODE_HPP
#define OBJECT_PICKUP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * @class ObjectPickupNode
 * @brief A ROS2 node that detects objects, navigates to them, and picks them up using TurtleBot3 and MoveIt.
 */
class ObjectPickupNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for ObjectPickupNode.
     */
    ObjectPickupNode();

private:
    /**
     * @brief Callback function for processing camera images.
     * @param msg Shared pointer to the received image message.
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Sends a goal to navigate the robot to the detected object.
     * @param target_pose The target pose to navigate to.
     */
    void navigateToObject(const geometry_msgs::msg::PoseStamped &target_pose);

    /**
     * @brief Executes the motion to pick up the detected object using MoveIt.
     */
    void pickUpObject();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;  ///< Subscription to the camera image topic.
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_; ///< Client for navigation action to move the robot.
};

#endif // OBJECT_PICKUP_NODE_HPP