// Including the required header files
#include "aruco_listener.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

// For using time literals (ms,s)
using namespace std::chrono_literals;

void FINAL::ArucoSub::aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr my_aruco)
{
  // Extracting the subscribed information from the /aruco_marker topic and printing it
    marker_id = my_aruco->marker_ids[0];
    RCLCPP_INFO_STREAM(this->get_logger(), "Marker Id :" <<marker_id);
    // Printing the aruco_position for reference
    RCLCPP_INFO_STREAM(this->get_logger(), "X :" <<my_aruco->poses[0].position.x);
    RCLCPP_INFO_STREAM(this->get_logger(), "Y :" <<my_aruco->poses[0].position.y);
    RCLCPP_INFO_STREAM(this->get_logger(), "Z :" <<my_aruco->poses[0].position.z);
    // Assigning parameters based on the value of the marker id
    if (marker_id==0)
    { 
        color[0]=get_parameter("aruco_0.wp1.color").as_string();
        color[1]=get_parameter("aruco_0.wp2.color").as_string();
        color[2]=get_parameter("aruco_0.wp3.color").as_string();
        color[3]=get_parameter("aruco_0.wp4.color").as_string();
        color[4]=get_parameter("aruco_0.wp5.color").as_string();
        // Attribute set to true once the marker is detected
        marker_detect=true;
    }
    else if (marker_id==1)
    {
        color[0]=get_parameter("aruco_1.wp1.color").as_string();
        color[1]=get_parameter("aruco_1.wp2.color").as_string();
        color[2]=get_parameter("aruco_1.wp3.color").as_string();
        color[3]=get_parameter("aruco_1.wp4.color").as_string();
        color[4]=get_parameter("aruco_1.wp5.color").as_string();
        marker_detect=true;

    }
    // Killing the aruco subscriber by resetting the object
    subscription_aruco.reset();
  
}

void FINAL::ArucoSub::subscriber1_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr my_msg)
{   
    // Creating an attribute to store the object detected
    std::string detected_color_1;
    // Calling the required function to detect the part based on part color and part type
    detected_color_1= detected_part(my_msg->part_poses[0].part.color, my_msg->part_poses[0].part.type);
    // Mapping the corresponding frame to the detected color
    battery_color_map[detected_color_1]="battery1_frame";
}

void FINAL::ArucoSub::subscriber2_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr my_msg)
{   
    // Creating an attribute to store the object detected
    std::string detected_color_2;
    detected_color_2= detected_part(my_msg->part_poses[0].part.color, my_msg->part_poses[0].part.type);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Detected color: "<<detected_color_2.c_str());
    // Mapping the corresponding frame to the detected color
    battery_color_map[detected_color_2]="battery2_frame";
}

void FINAL::ArucoSub::subscriber3_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr my_msg)
{   
    // Creating an attribute to store the object detected
    std::string detected_color_3;
    detected_color_3= detected_part(my_msg->part_poses[0].part.color, my_msg->part_poses[0].part.type);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Detected color: "<<detected_color_3.c_str());
    // Mapping the corresponding frame to the detected color
    battery_color_map[detected_color_3]="battery3_frame";
}

void FINAL::ArucoSub::subscriber4_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr my_msg)
{   
    // Creating an attribute to store the object detected
    std::string detected_color_4;
    detected_color_4= detected_part(my_msg->part_poses[0].part.color, my_msg->part_poses[0].part.type);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Detected color: "<<detected_color_4.c_str());
    // Mapping the corresponding frame to the detected color
    battery_color_map[detected_color_4]="battery4_frame";
}

void FINAL::ArucoSub::subscriber5_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr my_msg)
{   
    // Creating an attribute to store the object detected
    std::string detected_color_5;
    detected_color_5= detected_part(my_msg->part_poses[0].part.color, my_msg->part_poses[0].part.type);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Detected color: "<<detected_color_5.c_str());
    // Mapping the corresponding frame to the detected color
    battery_color_map[detected_color_5]="battery5_frame";
    // Printing the detected colors and the corresponding frames
    std::map<std::string, std::string>::iterator it = battery_color_map.begin();
    while(it != battery_color_map.end()){
    RCLCPP_INFO_STREAM(this->get_logger(),"Color: "<<it->first<<", Frame: "<<it->second);
    ++it;
    }
    // Declaring a geometry_pose object to store transforms 
    geometry_msgs::msg::Pose pose_out;
    // Checking if the marker is detected and the mapping is complete
    if (marker_detect==true && battery_color_map.size()>4)
    {   
        // For each color the transformation is made from the camera frame to the map frame by getting the value of the corresponding frame from the map
        // After the transfromation is performed the new pose and orientation is mapped to the respective color
        // This is done for all the batteries
        frame1 = battery_color_map[color[0]];
        pose_out = listen_transform("map",frame1);
        battery_frame_map[color[0]] = pose_out;

        frame2 = battery_color_map[color[1]];
        pose_out = listen_transform("map",frame2);
        battery_frame_map[color[1]] = pose_out;
        
        frame3 = battery_color_map[color[2]];
        pose_out = listen_transform("map",frame3);
        battery_frame_map[color[2]] = pose_out;
        
        frame4 = battery_color_map[color[3]];
        pose_out = listen_transform("map",frame4);
        battery_frame_map[color[3]] = pose_out;
        
        frame5 = battery_color_map[color[4]];
        pose_out = listen_transform("map",frame5);
        battery_frame_map[color[4]] = pose_out;
     
        RCLCPP_INFO_STREAM(this->get_logger(),"------------------------------------------------");
        // Storing the poses of each battery in the respective order for navigation
        unsigned int j=0;
        while(j<battery_frame_map.size())
        {   
            waypoints.push_back(battery_frame_map[color[j]]);
            // Printing the waypoints in the order to be navigated
            RCLCPP_INFO_STREAM(this->get_logger(),"Color: "<<color[j]<<" Pose: "<<waypoints[j].position.x<<", "
                                                            <<waypoints[j].position.y<<", "<<waypoints[j].position.z);
            j++;
        }
        // Stoping all the unwanted subscribers by reseting the object
        battery_subscription1_.reset();
        battery_subscription2_.reset();
        battery_subscription3_.reset();
        battery_subscription4_.reset();
        battery_subscription5_.reset();
        set_initial_pose();
        send_goal(waypoints);
    }

}

void FINAL::ArucoSub::send_goal(std::vector<geometry_msgs::msg::Pose> waypoints)
{
  using namespace std::placeholders;
  if (!this->client_->wait_for_action_server()) 
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }
  auto goal_msg = FollowWaypoints::Goal();
  geometry_msgs::msg::PoseStamped pose;
  // Extracting and sending the waypoints
  for (unsigned int i =0; i<waypoints.size(); i++)
  {
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose=waypoints[i];
    goal_msg.poses.push_back(pose);
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  //Performing navigation
  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&ArucoSub::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&ArucoSub::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&ArucoSub::result_callback, this, _1); std::cout<<"m here8"<<std::endl;
  client_->async_send_goal(goal_msg, send_goal_options);

}

void FINAL::ArucoSub::set_initial_pose()
{
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

  tf2::Quaternion q_test;
  q_test.setRPY(0.0, 0.0, -1.5707);
  message.pose.pose.orientation = tf2::toMsg(q_test);
  message.header.frame_id = "map";
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = -1.59;
  // message.pose.pose.orientation.x = 0.0;
  // message.pose.pose.orientation.y = 0.0;
  // message.pose.pose.orientation.z = -0.7070727;
  // message.pose.pose.orientation.w = 0.7071408;
  const std::chrono::seconds duration(5);
  auto start_time = std::chrono::steady_clock::now();
  std::cout<<"m inside the set_init_pose1"<<std::endl;
  while (std::chrono::steady_clock::now() - start_time < duration) 
  {  
    initial_pose_pub_->publish(message);
  }
  std::cout<<"m inside the set_init_pose2"<<std::endl;
}

void FINAL::ArucoSub::goal_response_callback
(
    // Declaring the callback
    std::shared_future<GoalHandleNavigation::SharedPtr> future) 
    {
        auto goal_handle = future.get();
        if (!goal_handle) 
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            rclcpp::shutdown();
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting for result");
        }
}

void FINAL::ArucoSub::feedback_callback(GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) 
{
    RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

void FINAL::ArucoSub::result_callback(const GoalHandleNavigation::WrappedResult& result) 
{
    // Switch case fro selecting the result
    switch (result.code) 
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
        default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    rclcpp::shutdown();
}

std::string FINAL::ArucoSub::detected_part(const unsigned int part_code, const unsigned int part_type)
{   
    // Determining the detected color based on the part code and  part type information from the message
    if (part_code == 0 && part_type ==  10)
        detected_color="red";
    else if (part_code == 1 && part_type == 10)
        detected_color="green";
    else if (part_code == 2 && part_type == 10)
        detected_color="blue";
    else if (part_code == 3 && part_type == 10)
        detected_color="orange";
    else if (part_code == 4 && part_type == 10)
        detected_color="purple";

    return detected_color;
}

geometry_msgs::msg::Pose FINAL::ArucoSub::listen_transform(const std::string &source_frame, const std::string &target_frame)
{   
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {   // Storing the transforms available in the buffer. Transform from aruco_frame to odom frame
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero);
    }
    // If no transfrom availbale print exception
    catch (const tf2::TransformException &ex)
    {   
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " 
                << target_frame << ": " << ex.what());
        return pose_out;
    }
    // Storing the required information from the transform
    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;
    RCLCPP_INFO_STREAM(this->get_logger(), "Transform done");
    
    // Returning the position and orientation of the target frame in the source frame
    return pose_out;

}
// Defining the main() with the node name
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Create an instance for the node
  auto node = std::make_shared<FINAL::ArucoSub>("aruco_listen");
  // Keeping the node active
  rclcpp::spin(node);
  rclcpp::shutdown();
}

