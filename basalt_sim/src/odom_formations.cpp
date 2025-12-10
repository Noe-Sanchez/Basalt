#include <chrono>
#include <iostream>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class Formations2 : public rclcpp::Node{
  public:
    Formations2(): Node("formations2_node"){
      // Drone parameters
      num_drones = this->declare_parameter("num_drones", 3);

      // Resize odoms
      follower_odom_msgs.resize(num_drones);
      follower_odoms.resize(num_drones);
      follower_tfs.resize(num_drones);

      // Initialize on takeoff formation
      for(int i = 0; i < num_drones; i++){
	follower_tfs.push_back(geometry_msgs::msg::TransformStamped());
	follower_tfs[i].header.frame_id = "leader";
	follower_tfs[i].child_frame_id = "follower_" + std::to_string(i+1);
	follower_tfs[i].transform.translation.x = (i * 2.0) - 2; // 1 meter apart in y direction
	follower_tfs[i].transform.translation.y = 0.0;
	follower_tfs[i].transform.translation.z = 0.0;
	follower_tfs[i].transform.rotation.x = 0.0;
	follower_tfs[i].transform.rotation.y = 0.0;
	follower_tfs[i].transform.rotation.z = 0.0;
	follower_tfs[i].transform.rotation.w = 1.0;
      }

      // Subscribers
      desired_formation_subscriber   = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/definition", 10, std::bind(&Formations2::formation_callback,     this, std::placeholders::_1));
      formation_dot_subscriber       = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/velocity",  10, std::bind(&Formations2::formation_dot_callback, this, std::placeholders::_1));
      desired_leader_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/leader/state",              10, std::bind(&Formations2::leader_pose_callback,   this, std::placeholders::_1));

      // Timer 
      //control_timer = this->create_wall_timer(50ms, std::bind(&Formations2::control_callback, this));
      control_timer = this->create_wall_timer(10ms, std::bind(&Formations2::control_callback, this));

      // Dynamic odom publishers
      for (int i = 0; i < num_drones; i++){
	std::string topic_name = "/control_" + std::to_string(i+1) + "/reference/pose";
        follower_odom_publishers.push_back(this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10));
        follower_odom_msgs.push_back(nav_msgs::msg::Odometry());
        follower_tfs.push_back(geometry_msgs::msg::TransformStamped());

      }

      // Declare tf_broadcaster
      tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);


    }

    void formation_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      for (int i = 0; i < num_drones; i++){
        follower_tfs[i].header.frame_id = "leader";
	follower_tfs[i].child_frame_id = "follower_" + std::to_string(i+1);
	follower_tfs[i].transform.translation.x = msg->poses[i].position.x;
	follower_tfs[i].transform.translation.y = msg->poses[i].position.y;
	follower_tfs[i].transform.translation.z = msg->poses[i].position.z;
	follower_tfs[i].transform.rotation = msg->poses[i].orientation;
      }
    }

    void leader_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      leader_tf.header = msg->header;
      leader_tf.header.frame_id = "world";
      leader_tf.child_frame_id = "leader";
      leader_tf.transform.translation.x = msg->pose.pose.position.x;
      leader_tf.transform.translation.y = msg->pose.pose.position.y;
      leader_tf.transform.translation.z = msg->pose.pose.position.z;
      leader_tf.transform.rotation = msg->pose.pose.orientation;
      leader_odom = *msg;
    }
    void formation_dot_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      // Currently not used, but can be implemented for velocity control
      geometry_msgs::msg::PoseArray formation_dot = *msg;
    }

    void control_callback(){

      tf_broadcaster->sendTransform(leader_tf);

      for(int i = 0; i < num_drones; i++){
	// Compute follower odometry based on leader odometry and desired formation
	follower_odom_msgs[i].header.frame_id = "world";
	follower_odom_msgs[i].header.stamp = this->now();
	// Position
	follower_odom_msgs[i].pose.pose.position.x = leader_tf.transform.translation.x + follower_tfs[i].transform.translation.x;
	follower_odom_msgs[i].pose.pose.position.y = leader_tf.transform.translation.y + follower_tfs[i].transform.translation.y;
	follower_odom_msgs[i].pose.pose.position.z = leader_tf.transform.translation.z + follower_tfs[i].transform.translation.z;
	// Orientation (simple addition, may need quaternion multiplication for real applications)
	follower_odom_msgs[i].pose.pose.orientation = leader_tf.transform.rotation;

	// Velocity (leader for now)
	follower_odom_msgs[i].twist.twist = leader_odom.twist.twist;

	// Publish follower odometry
	follower_odom_publishers[i]->publish(follower_odom_msgs[i]);

	// Publish tfs
        follower_tfs[i].header.stamp = this->now();
	tf_broadcaster->sendTransform(follower_tfs[i]);
      }
    }

  private:
    float num_drones;

    nav_msgs::msg::Odometry leader_odom;
    geometry_msgs::msg::TransformStamped leader_tf;
    std::vector<geometry_msgs::msg::TransformStamped> follower_tfs;
    std::vector<nav_msgs::msg::Odometry> follower_odom_msgs;


    std::vector<nav_msgs::msg::Odometry> follower_odoms;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr     desired_formation_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr     formation_dot_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           desired_leader_pose_subscriber;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> follower_odom_publishers;
    rclcpp::TimerBase::SharedPtr control_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;


};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Formations2>());
  rclcpp::shutdown();
  return 0;
}
