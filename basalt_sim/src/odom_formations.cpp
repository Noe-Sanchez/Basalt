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

using namespace std::chrono_literals;

class Formations2 : public rclcpp::Node{
  public:
    Formations2(): Node("formations2_node"){
      // Drone parameters
      num_drones = this->declare_parameter("num_drones", 3);

      // Subscribers
      desired_formation_subscriber   = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/defintion", 10, std::bind(&Formations2::formation_callback,     this, std::placeholders::_1));
      formation_dot_subscriber       = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/velocity",  10, std::bind(&Formations2::formation_dot_callback, this, std::placeholders::_1));
      desired_leader_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/leader/state",              10, std::bind(&Formations2::leader_pose_callback,   this, std::placeholders::_1));

      // Timer 
      control_timer = this->create_wall_timer(20ms, std::bind(&Formations2::control_callback, this));

      // Dynamic odom publishers
      for (int i = 0; i < num_drones; i++){
	std::string topic_name = "/control_" + std::to_string(i+1) + "/reference/pose";
        follower_odom_publishers.push_back(this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10));
        follower_odom_msgs.push_back(nav_msgs::msg::Odometry());
        follower_tfs.push_back(geometry_msgs::msg::TransformStamped());

      }

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
      leader_tf.child_frame_id = "leader";
      leader_tf.transform.translation.x = msg->pose.pose.position.x;
      leader_tf.transform.translation.y = msg->pose.pose.position.y;
      leader_tf.transform.translation.z = msg->pose.pose.position.z;
      leader_tf.transform.rotation = msg->pose.pose.orientation;
    }

    void control_callback(){
    }

  private:
    float num_drones;

    nav_msgs::msg::Odometry leader_odom;

    std::vector<nav_msgs::msg::Odometry> follower_odoms;
    Eigen::Matrix3f Rq_leader;


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr     desired_formation_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           desired_leader_pose_subscriber;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> follower_odom_publishers;
    rclcpp::TimerBase::SharedPtr control_timer;


};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Formations2>());
  rclcpp::shutdown();
  return 0;
}
