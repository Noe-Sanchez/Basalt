#include <chrono>
#include <iostream>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class Formations2 : public rclcpp::Node{
  public:
    Formations2(): Node("formations2_node"){
      // Drone parameters
      num_drones     = this->declare_parameter("num_drones",     12);
      row_separation = this->declare_parameter("row_separation", 2.0);
      max_rows       = this->declare_parameter("max_rows",       4);
      leader_number  = this->declare_parameter("leader_number",  -1);

      // Resize odoms
      follower_odom_publishers.resize(num_drones);
      follower_odoms.resize(num_drones);
      follower_tfs.resize(num_drones);
      gammas.resize(num_drones);
      gammas_dot.resize(num_drones);
      gammas_ddot.resize(num_drones);

      // CF
      cfs_positions.resize(num_drones);
      cfs_positions_publishers.resize(num_drones);

      // Initialize on takeoff formation
      for(int i = 0; i < num_drones; i++){
	//follower_tfs.push_back(geometry_msgs::msg::TransformStamped());
	follower_tfs[i] = geometry_msgs::msg::TransformStamped();
	follower_tfs[i].header.frame_id = "leader";
	follower_tfs[i].child_frame_id = "follower_" + std::to_string(i+1);
	follower_tfs[i].transform.translation.x = (i % max_rows) * row_separation; 
	follower_tfs[i].transform.translation.y = (i / max_rows) * row_separation;
	follower_tfs[i].transform.translation.z = 0.0;
	follower_tfs[i].transform.rotation.x = 0.0;
	follower_tfs[i].transform.rotation.y = 0.0;
	follower_tfs[i].transform.rotation.z = 0.0;
	follower_tfs[i].transform.rotation.w = 1.0;

	gammas[i] << follower_tfs[i].transform.translation.x, 
	             follower_tfs[i].transform.translation.y, 
		     follower_tfs[i].transform.translation.z;
      }

      // Subscribers
      desired_formation_subscriber   = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/definition", 10, std::bind(&Formations2::formation_callback,     this, std::placeholders::_1));
      formation_dot_subscriber       = this->create_subscription<geometry_msgs::msg::PoseArray>("/formation/velocity",   10, std::bind(&Formations2::formation_dot_callback, this, std::placeholders::_1));
      //desired_leader_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(      "/leader/state",         10, std::bind(&Formations2::leader_pose_callback,   this, std::placeholders::_1));
      desired_leader_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(      "/control_1/diff/odom",         10, std::bind(&Formations2::leader_pose_callback,   this, std::placeholders::_1));

      // Timer 
      //control_timer = this->create_wall_timer(50ms, std::bind(&Formations2::control_callback, this));
      control_timer = this->create_wall_timer(10ms, std::bind(&Formations2::control_callback, this));

      // Dynamic odom publishers
      for (int i = 0; i < num_drones; i++){
        if (i == leader_number) continue; // Skip leader if specified
	std::string topic_name = "/control_" + std::to_string(i+1) + "/reference/pose";
        //follower_odom_publishers.push_back(this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10));
        //follower_odoms.push_back(nav_msgs::msg::Odometry());
        
	std::string cf_name = "/cf" + std::to_string(i+1) + "/cmd_position";
	//cfs_positions.push_back(crazyflie_interfaces::msg::Position());
	//cfs_positions_publishers.push_back(this->create_publisher<crazyflie_interfaces::msg::Position>(cf_name, 10));
	
	//follower_tfs.push_back(geometry_msgs::msg::TransformStamped());

	//gammas.push_back(       Eigen::Vector3d(0.0, 0.0, 0.0));
	//gammas_dot.push_back(   Eigen::Vector3d(0.0, 0.0, 0.0));
	//gammas_ddot.push_back(  Eigen::Vector3d(0.0, 0.0, 0.0));
	
	// Set to index instead of push back, since we resized
	follower_odom_publishers[i] = this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
	follower_odoms[i] = nav_msgs::msg::Odometry();
	cfs_positions_publishers[i] = this->create_publisher<crazyflie_interfaces::msg::Position>(cf_name, 10);
	cfs_positions[i] = crazyflie_interfaces::msg::Position();

	//gammas[i] = Eigen::Vector3d(0.0, 0.0, 0.0);
	gammas_dot[i]  = Eigen::Vector3d(0.0, 0.0, 0.0);
	gammas_ddot[i] = Eigen::Vector3d(0.0, 0.0, 0.0);

      }

      // Calc vars
      leader_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      leader_cross       = Eigen::Matrix3d::Zero();
      lambda             << 0.0, 0.0, 0.0;
      lambda_dot         << 0.0, 0.0, 0.0;
      lambda_ddot        << 0.0, 0.0, 0.0;

      // Declare tf_broadcaster
      tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);


    }

    void formation_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      for (int i = 0; i < num_drones; i++){
	if (i == leader_number) continue; // Skip leader if specified
        follower_tfs[i].header.frame_id         = "leader";
	follower_tfs[i].child_frame_id          = "follower_" + std::to_string(i+1);
	follower_tfs[i].transform.translation.x = msg->poses[i].position.x;
	follower_tfs[i].transform.translation.y = msg->poses[i].position.y;
	follower_tfs[i].transform.translation.z = msg->poses[i].position.z;
	follower_tfs[i].transform.rotation      = msg->poses[i].orientation;

        gammas[i] << msg->poses[i].position.x, 
	             msg->poses[i].position.y, 
		     msg->poses[i].position.z;
      }
    }

    void leader_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      leader_tf.header                  = msg->header;
      leader_tf.header.frame_id         = "world";
      leader_tf.child_frame_id          = "leader";
      leader_tf.transform.translation.x = msg->pose.pose.position.x;
      leader_tf.transform.translation.y = msg->pose.pose.position.y;
      leader_tf.transform.translation.z = msg->pose.pose.position.z;
      leader_tf.transform.rotation      = msg->pose.pose.orientation;

      // Rotate leader velocities to world frame, since they come from odom
      Eigen::Quaterniond leader_quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      Eigen::Vector3d leader_vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
      Eigen::Vector3d leader_vel_world = leader_quat * leader_vel;

      leader_odom = *msg;

      leader_odom.twist.twist.linear.x = leader_vel_world.x();
      leader_odom.twist.twist.linear.y = leader_vel_world.y();
      leader_odom.twist.twist.linear.z = leader_vel_world.z();
    }
    void formation_dot_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      // Currently not used, but can be implemented for velocity control
      geometry_msgs::msg::PoseArray formation_dot = *msg;
    }

    void control_callback(){

      tf_broadcaster->sendTransform(leader_tf);

      for(int i = 0; i < num_drones; i++){
	if (i == leader_number) continue; // Skip leader if specified
        // Get leader rotation
	leader_orientation.w() = leader_tf.transform.rotation.w;
	leader_orientation.x() = leader_tf.transform.rotation.x;
	leader_orientation.y() = leader_tf.transform.rotation.y;
	leader_orientation.z() = leader_tf.transform.rotation.z;

	leader_cross << 0.0, -leader_odom.twist.twist.angular.z,  leader_odom.twist.twist.angular.y,
		     	leader_odom.twist.twist.angular.z, 0.0, -leader_odom.twist.twist.angular.x,
		       -leader_odom.twist.twist.angular.y, leader_odom.twist.twist.angular.x, 0.0;
	//std::cout << "Leader cross:\n" << leader_cross << std::endl;

        // Compute lambda vectors for formation (only position for now)
	lambda      = leader_orientation * gammas[i]; 
	lambda_dot  = leader_orientation * gammas_dot[i] + leader_cross * lambda;

	std::cout << "Lambda for follower " << i+1 << ": (" << lambda.x() << ", " 
		  << lambda.y() << ", " 
		  << lambda.z() << ")" << std::endl;
	
	// Compute follower odometry based on leader odometry and desired formation
	follower_odoms[i].header.frame_id = "world";
	follower_odoms[i].header.stamp = this->now();
	// Position
	follower_odoms[i].pose.pose.position.x  = leader_odom.pose.pose.position.x + lambda.x();
	follower_odoms[i].pose.pose.position.y  = leader_odom.pose.pose.position.y + lambda.y();
	follower_odoms[i].pose.pose.position.z  = leader_odom.pose.pose.position.z + lambda.z();
	// Orientation (simple addition, may need quaternion multiplication for real applications)
	follower_odoms[i].pose.pose.orientation = leader_tf.transform.rotation;

	std::cout << "Follower " << i+1 << " position: (" << follower_odoms[i].pose.pose.position.x << ", " 
		  << follower_odoms[i].pose.pose.position.y << ", " 
		  << follower_odoms[i].pose.pose.position.z << ")" << std::endl;

	// Add velocities
	//follower_odoms[i].twist.twist.linear.x  = leader_odom.twist.twist.linear.x;
	//follower_odoms[i].twist.twist.linear.y  = leader_odom.twist.twist.linear.y;
	//follower_odoms[i].twist.twist.linear.z  = leader_odom.twist.twist.linear.z;
	follower_odoms[i].twist.twist.linear.x  = leader_odom.twist.twist.linear.x + lambda_dot.x();
	follower_odoms[i].twist.twist.linear.y  = leader_odom.twist.twist.linear.y + lambda_dot.y();
	follower_odoms[i].twist.twist.linear.z  = leader_odom.twist.twist.linear.z + lambda_dot.z();

	cfs_positions[i].x = follower_odoms[i].pose.pose.position.x;
	cfs_positions[i].y = follower_odoms[i].pose.pose.position.y;
	cfs_positions[i].z = follower_odoms[i].pose.pose.position.z;
	cfs_positions[i].yaw = 0.0;
	cfs_positions[i].header.stamp = this->now();

	cfs_positions_publishers[i]->publish(cfs_positions[i]);

	// Publish follower odometry
	follower_odom_publishers[i]->publish(follower_odoms[i]);

	// Publish tfs
        follower_tfs[i].header.stamp = this->now();
	tf_broadcaster->sendTransform(follower_tfs[i]);
      }
    }

  private:
    float num_drones;
    float row_separation;
    int   max_rows;
    int   leader_number;

    Eigen::Quaterniond           leader_orientation; // Characterizes leader rotation
    Eigen::Vector3d              lambda;             // Formation desired position
    Eigen::Vector3d              lambda_dot;         // Formation desired velocity
    Eigen::Vector3d              lambda_ddot;        // Formation desired acceleration
    Eigen::Matrix3d              leader_cross;       // Cross product matrix
    std::vector<Eigen::Vector3d> gammas;             // Formation pose definition
    std::vector<Eigen::Vector3d> gammas_dot;         // Formation velocity definition
    std::vector<Eigen::Vector3d> gammas_ddot;        // Formation acceleration definition

    nav_msgs::msg::Odometry                           leader_odom;
    geometry_msgs::msg::TransformStamped              leader_tf;
    std::vector<geometry_msgs::msg::TransformStamped> follower_tfs;
    std::vector<nav_msgs::msg::Odometry>              follower_odoms;
    std::vector<crazyflie_interfaces::msg::Position>  follower_positions;
    std::vector<crazyflie_interfaces::msg::Position>  cfs_positions;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr     desired_formation_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr     formation_dot_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           desired_leader_pose_subscriber;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> follower_odom_publishers;
    std::vector<rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr> cfs_positions_publishers;
    rclcpp::TimerBase::SharedPtr control_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;


};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Formations2>());
  rclcpp::shutdown();
  return 0;
}
