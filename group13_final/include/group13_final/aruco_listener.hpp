#pragma once
// Including the required header files
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory.h>
#include <vector>
#include <string>
#include <map>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
/**
 * @brief Namespace for the class ArucoSub
 * 
 */
namespace FINAL {
  /**
   * @brief Class for getting the aruco marker id and pose
   * 
   */
class ArucoSub : public rclcpp::Node {
 public:
 // Using the following namespaces
 using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
 using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
 /**
  * @brief Construct a new ArucoSub object
  * 
  * @param node_name aruco_listen
  */
  ArucoSub(std::string node_name) : Node(node_name) {
    RCLCPP_INFO(this->get_logger(), "Subscribing started");
    // Subscribing to the topic /aruco_marker to obtain the marker_id
    subscription_aruco = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 
    10, std::bind(&ArucoSub::aruco_sub_cb, this, std::placeholders::_1)); 
    // Subscribing to the topic /image/camera1/image to obtain the message from camera1
    // Similarly subscribing to differenr camera topics
    battery_subscription1_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera1/image", //camera1
    rclcpp::SensorDataQoS(), std::bind(&ArucoSub::subscriber1_cb_, this, std::placeholders::_1)); 
    battery_subscription2_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera2/image", //camera2
    rclcpp::SensorDataQoS(), std::bind(&ArucoSub::subscriber2_cb_, this, std::placeholders::_1)); 
    battery_subscription3_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera3/image", //camera3
    rclcpp::SensorDataQoS(), std::bind(&ArucoSub::subscriber3_cb_, this, std::placeholders::_1));         
    battery_subscription4_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera4/image", //camera4
    rclcpp::SensorDataQoS(), std::bind(&ArucoSub::subscriber4_cb_, this, std::placeholders::_1));         
    battery_subscription5_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera5/image", //camera5
    rclcpp::SensorDataQoS(), std::bind(&ArucoSub::subscriber5_cb_, this, std::placeholders::_1)); 
    //Load a buffer of transforms
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // initialize the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Declaring parameters
    for (const auto& point : wpoints)
    {
      const std::string parameter1 = "aruco_0." + point;
      this->declare_parameter(parameter1 + ".battery","battery");
      this->declare_parameter(parameter1 + ".color","no color");

      const std::string parameter2 = "aruco_1." + point;
      this->declare_parameter(parameter2 + ".type","battery");
      this->declare_parameter(parameter2 + ".color","no color");
    }
    //Initializing the client
    client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    // Creating a publisher to the topic /initialpose to initialise the robot
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);


  }
  private: 
    /**
     * @brief Pointer object for the aruco marker subscriber
     * 
     */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_aruco;
    /**
     * @brief Pointer objects for the camera1, camera2, camera3, camera4, camera5 subscribers
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription1_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription2_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription3_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription4_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription5_;    
    /**
     * @brief Creating a buffer onject for transformations
     * 
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /**
     * @brief Creating a broadcaster object
     * 
     */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::TimerBase::SharedPtr listen_timer_;
    /**
     * @brief Creating a pointer for the transform listener
     * 
     */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    /**
     * @brief False initialization of marker_id to prevent false values. If there is an aruco_marker with value 10 it should be changed
     * 
     */
    int marker_id = 10;
    /**
     * @brief Creating a vector of strings
     * 
     */
    const std::vector<std::string> wpoints = {"wp1","wp2", "wp3", "wp4", "wp5"};
    /**
     * @brief Creating a vector of geometry_msgs to store the waypoints for navigation in the required order
     * 
     */
    std::vector<geometry_msgs::msg::Pose> waypoints;
    /**
     * @brief Creating a string to store the different color values from the parameters
     * 
     */
    std::string color[5];
    /**
     * @brief Creating an attribute to store the detected color using the camera
     * 
     */
    std::string detected_color;
    /**
     * @brief Creating attribute to store the battery_names
     * All the following attributes frame1, frame2, frame3, frame4, frame5 does the same thing
     */
    std::string frame1;
    std::string frame2;
    std::string frame3;
    std::string frame4;
    std::string frame5;
    /**
     * @brief Creating a map to store the batttery color and the corresponding frame number
     * 
     */
    std::map<std::string, std::string> battery_color_map;
    /**
     * @brief Creating a map to store the color and respective transforations in the map frame
     * 
     */
    std::map<std::string, geometry_msgs::msg::Pose> battery_frame_map;
    /**
     * @brief An attribute to check whether the aruco_marker is detected
     * 
     */
    bool marker_detect = false;
    /**
     * @brief Call back function for the /aruco_marker topic subscriber
     * 
     * @param my_aruco pointer object to the message 
     */
    void aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr my_aruco); 
    /**
     * @brief Subscriber calll back function to topics of the following cameras:
     * camera1, camera2, camera3, camera4, camera5
     * @param msg Pointer to the object
     */
    void subscriber1_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void subscriber2_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void subscriber3_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void subscriber4_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void subscriber5_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief Function to do the transformation between two frames
     * 
     * @param source_frame Frame about which the transformation is performed
     * @param target_frame Frame on which the transformation is performed
     * @return geometry_msgs::msg::Pose The corresponding pose in the map frame 
     */
    geometry_msgs::msg::Pose listen_transform(const std::string &source_frame, const std::string &target_frame);
    /**
     * @brief Function to determine the color of the object based on the part code and type obtained from logical camera message
     * 
     * @param part_code Part code of the object found
     * @param part_type Type of the object found
     * @return std::string The detected color
     */
    std::string detected_part(const unsigned int part_code, const unsigned int part_type);
     /**
     * @brief Creating a publisher object for publishing initial pose
     *
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    /**
     * @brief Creating an action client for the action server navigate_to_pose
     *
     */
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
    /**
     * @brief Response from the server after sending the goal
     */
    void goal_response_callback(
        std::shared_future<GoalHandleNavigation::SharedPtr> future);
    /**
     * @brief Feedback received while the robot is driving towards the goal
     *
     * @param feedback
     */
    void feedback_callback(
        GoalHandleNavigation::SharedPtr,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    /**
     * @brief Logs the result after the action has completed
     *
     * @param result
     */
    void result_callback(const GoalHandleNavigation::WrappedResult& result);
    /**
     * @brief sets the initial pose of the robot by publishing the initial pose on the topic initpose for 5sec.
     * 
     */
    void set_initial_pose();
    /**
     * @brief Publishes a given waypoint of geometry poses on the action to make robot move to those points
     * 
     * @param waypoints points for navigation
     */
    void send_goal(std::vector<geometry_msgs::msg::Pose> waypoints);



};  // class ArucoSub
}  // namespace FINAL`