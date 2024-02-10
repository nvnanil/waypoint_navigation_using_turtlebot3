#pragma once
// Including all the required files
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <mage_msgs/msg/part_pose.hpp>
// For using time literals (ms,s)
using namespace std::chrono_literals;
/**
 * @brief Namespace for the broadcaster class
 * 
 */
namespace FINAL
{
/**
 * @brief Class for broadcasting the pose of the battery in the camera frame
 * 
 */
class BroadcasterDemo : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Broadcaster Demo object
     * 
     * @param node_name broadcaster_demo
     */
    BroadcasterDemo(std::string node_name) : Node(node_name)
    {
        // Starting the broadcaster
        RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Setting dedicated thread
        tf_buffer_->setUsingDedicatedThread(true);

        // Subscribing to the camera topics for extracting the pose of the battery
        // Camera topics for camera1, camera2, camera3, camera4, camera5
        battery_subscription1_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera1/image", 
        rclcpp::SensorDataQoS(), std::bind(&BroadcasterDemo::broadcast_timer1_cb_, this, std::placeholders::_1)); 
        battery_subscription2_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera2/image", 
        rclcpp::SensorDataQoS(), std::bind(&BroadcasterDemo::broadcast_timer2_cb_, this, std::placeholders::_1)); 
        battery_subscription3_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera3/image", 
        rclcpp::SensorDataQoS(), std::bind(&BroadcasterDemo::broadcast_timer3_cb_, this, std::placeholders::_1)); 
        battery_subscription4_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera4/image", 
        rclcpp::SensorDataQoS(), std::bind(&BroadcasterDemo::broadcast_timer4_cb_, this, std::placeholders::_1)); 
        battery_subscription5_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera5/image", 
        rclcpp::SensorDataQoS(), std::bind(&BroadcasterDemo::broadcast_timer5_cb_, this, std::placeholders::_1)); 
    }


private:
    /**
     * @brief x_pose, y_pose, z_pose stores the position of the robot
     * 
     */
    float x_pose;
    float y_pose;
    float z_pose;
    /**
     * @brief x_angle, y_angle, z_angle, w_angle stores the orientation of the robot in Quaternion
     * 
     */
    float x_angle;
    float y_angle;
    float z_angle;
    float w_angle;
    /**
     * @brief Buffer that stores several seconds of transforms for easy lookup by the listener
     * 
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /**
     * @brief Broadcaster object for the transforms
     * 
     */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /**
     * @brief Pointer object to the following camera topics for extracting the pose of the battery
     * camera1, camera2, camera3, camera4, camera5
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription1_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription2_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription3_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription4_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription5_; 
    /**
     * @brief Subscriber calll back function to topics of the following cameras:
     * camera1, camera2, camera3, camera4, camera5
     * @param msg Pointer to the object
     */
    void broadcast_timer1_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void broadcast_timer2_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void broadcast_timer3_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void broadcast_timer4_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void broadcast_timer5_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);


}; // class Broadcasterdemo
} // namespace FINAL