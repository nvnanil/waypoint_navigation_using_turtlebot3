// Including the necessory header files
#include "broadcaster_demo.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

// Allows to use, 50ms, etc
using namespace std::chrono_literals;

void FINAL::BroadcasterDemo::broadcast_timer1_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    // Defining the source frame and the target frame fro the transformation
    std::string source_frame = "camera1_frame";
    std::string target_frame = "battery1_frame";
    // std::string tf_broadcaster_ = "tf_broadcaster"+std::to_string(i)+"_";
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = source_frame;
    dynamic_transform_stamped.child_frame_id = target_frame;

    dynamic_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    dynamic_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    dynamic_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;
    // Sending the obtained dynamic transfrom
    tf_broadcaster_->sendTransform(dynamic_transform_stamped);

}

void FINAL::BroadcasterDemo::broadcast_timer2_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    std::string source_frame = "camera2_frame";
    std::string target_frame = "battery2_frame";
    // std::string tf_broadcaster_ = "tf_broadcaster"+std::to_string(i)+"_";
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = source_frame;
    dynamic_transform_stamped.child_frame_id = target_frame;

    dynamic_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    dynamic_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    dynamic_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    tf_broadcaster_->sendTransform(dynamic_transform_stamped);

}

void FINAL::BroadcasterDemo::broadcast_timer3_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    std::string source_frame = "camera3_frame";
    std::string target_frame = "battery3_frame";
    // std::string tf_broadcaster_ = "tf_broadcaster"+std::to_string(i)+"_";
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = source_frame;
    dynamic_transform_stamped.child_frame_id = target_frame;

    dynamic_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    dynamic_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    dynamic_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    tf_broadcaster_->sendTransform(dynamic_transform_stamped);

}

void FINAL::BroadcasterDemo::broadcast_timer4_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    std::string source_frame = "camera4_frame";
    std::string target_frame = "battery4_frame";
    // std::string tf_broadcaster_ = "tf_broadcaster"+std::to_string(i)+"_";
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = source_frame;
    dynamic_transform_stamped.child_frame_id = target_frame;

    dynamic_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    dynamic_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    dynamic_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    tf_broadcaster_->sendTransform(dynamic_transform_stamped);

}

void FINAL::BroadcasterDemo::broadcast_timer5_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    std::string source_frame = "camera5_frame";
    std::string target_frame = "battery5_frame";
    // std::string tf_broadcaster_ = "tf_broadcaster"+std::to_string(i)+"_";
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = source_frame;
    dynamic_transform_stamped.child_frame_id = target_frame;

    dynamic_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    dynamic_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    dynamic_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    tf_broadcaster_->sendTransform(dynamic_transform_stamped);

}
// Defining the main() with the node name
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Creating an instance for the node
  auto node = std::make_shared<FINAL::BroadcasterDemo>("broadcaster_demo");
  // Keping the node alive
  rclcpp::spin(node);
  rclcpp::shutdown();
}