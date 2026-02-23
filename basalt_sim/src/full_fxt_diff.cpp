#include <chrono>
#include <iostream>
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench.hpp"

// Algorithmic class for fullfxttd, since operations are mostly element-wise
// Variables and methods for 1 DOF independent differentiation
class FXTTD {
  public:
    FXTTD(){}; // Default constructor, do nothing
    //FXTTD(double _dt=0.05, double _z_1=0.0, double _x_1=0.0){
    FXTTD(double _dt=0.05, double _z_1=0.0){
      // Init algorithmic
      dt               =  _dt;
      e_diff           =  0.0;
      //x_1              = _x_1; 
      z_1              = _z_1; 
      z_1_dot          =  0.0;
      z_1_dot_previous =  0.0;
      z_2              =  0.0;
      z_2_dot          =  0.0;
      z_2_dot_previous =  0.0;
      z_3              =  0.0;
      z_3_dot          =  0.0;
      z_3_dot_previous =  0.0;
	
      // Init gains
      epsilon1 = 0.5;
      epsilon2 = 0.5;
      epsilon3 = 0.5;
      alpha    = 0.5;
      beta     = 1.5;
    }
    // Setters
    void setGains(double _epsilon1, double _epsilon2, double _epsilon3, double _alpha, double _beta){
      epsilon1 = _epsilon1;
      epsilon2 = _epsilon2;
      epsilon3 = _epsilon3;
      alpha    = _alpha;
      beta     = _beta;
    }
    // We wont be setting odom because of the manual error handling
    //void setOdom(double _x_1){
    //  x_1 = _x_1; 
    //}
    // Manual error setter, because of special quaternion error handling
    void setError(double _e_diff){ 
      e_diff = _e_diff;
    }
    // Getters
    double get_z1(){
      return z_1;
    }
    double get_z2(){
      return z_2;
    }
    double get_z3(){
      return z_3;
    }


    // Main algo, extract before recompute, run manually on correct dt
    void compute(){
      // Compute error
      // e_diff = x_1 - z_1;
      
      // We would normally compute error by every DOF, but since quaternion math is strange, the error will be fed as well

      // Third state
      z_3_dot = epsilon3*(sig(e_diff, alpha) + sig(e_diff, beta));
      // Trapezoidal integration
      z_3 = z_3 + (z_3_dot + z_3_dot_previous)*0.5*dt;
      z_3_dot_previous = z_3_dot;

      // Second state
      z_2_dot = z_3 + epsilon2*(sig(e_diff, (alpha + 1.0)/2.0) + sig(e_diff, (beta + 1.0)/2.0));
      // Trapezoidal integration
      z_2 = z_2 + (z_2_dot + z_2_dot_previous)*0.5*dt;
      z_2_dot_previous = z_2_dot;
      
      // First state
      z_1_dot = z_2 + epsilon1*(sig(e_diff, (alpha + 2.0)/3.0) + sig(e_diff, (beta + 2.0)/3.0));
      // Trapezoidal integration
      z_1 = z_1 + (z_1_dot + z_1_dot_previous)*0.5*dt;
      z_1_dot_previous = z_1_dot;

      // Variables are now exposed, extract z_1, z_2, and z_3 as estimates
    }


  private: 
    // Internal implementation of sign
    double sign(double x){
      if (x > 0) {
        return 1.0;
      } else if (x < 0) {
        return -1.0;
      } else {
        return 0.0;
      }
    }
    // Internal implementation of sig
    double sig(double x, double exp){
      return sign(x) * pow(fabs(x), exp);
    }

    // Algorithmic variables
    double dt;               // Time step for integration
    double e_diff;           // This DOF error component
    //double x_1;              // This DOF odom input
    double z_1;              // First degree diff estimate
    double z_1_dot;          // First degree diff estimate derivative
    double z_1_dot_previous; // First degree diff estimate derivative previous
    double z_2;              // Second degree diff estimate
    double z_2_dot;          // Second degree diff estimate derivative
    double z_2_dot_previous; // Second degree diff estimate derivative previous
    double z_3;              // Third degree diff estimate
    double z_3_dot;          // Third degree diff estimate derivative
    double z_3_dot_previous; // Third degree diff estimate derivative previous
    
    // Gains
    double epsilon1;         // First estimation gain 
    double epsilon2;         // Second estimation gain
    double epsilon3;         // Third estimation gain
    double alpha;            // Slow dynamics exponent gain
    double beta;             // Fast dynamics exponent gain

};

  

using namespace std::chrono_literals;

class FXTTD_Node : public rclcpp::Node{
  public:
    FXTTD_Node(): Node("fxttd_node"){
      // Namespace for tf
      this->declare_parameter<std::string>("tf_namespace", "x500_1");

      // Subscribers
      sim_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/model/x500_1/odometry", 10, std::bind(&FXTTD_Node::sim_pose_callback,     this, std::placeholders::_1));

      // Publishers
      diff_pose_publisher  = this->create_publisher<nav_msgs::msg::Odometry>("/control_1/diff/odom", 10);
      diff_error_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/control_1/diff/error", 10);

      // Tf
      tf_broadcaster      = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      // Timer
      control_timer       = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&FXTTD_Node::control_callback, this));

      // Init precalculated 
      sim_pose   = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_vel    = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_omega  = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_quat   = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      // Init precalculated, helpers
      vel_body   = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      vel_world  = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

      // Resize to DOFs
      e_diff.resize(6);
      z_1.resize(6);
      z_2.resize(6);
      z_3.resize(6);
      epsilon1.resize(6);
      epsilon2.resize(6);
      epsilon3.resize(6);
      alpha.resize(6);
      beta.resize(6);

      // Init algorithmic variables
      e_diff     << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
      z_1        << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;               
      z_2        << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;               
      z_3        << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;               
      q_hat      = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      q_e        = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
			
      // Init gains
      epsilon1   << 60.0, 60.0, 80.0, 60.0, 60.0, 60.0; // First estimation gain
      epsilon2   << 60.0, 60.0, 80.0, 180.0, 180.0, 180.0; // Second estimation gain
      epsilon3   << 60.0, 60.0, 80.0, 120.0, 120.0, 120.0; // Third estimation gain
      //epsilon3   << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0; // Third estimation gain
      //alpha      << 0.75, 0.75, 0.75, 0.95, 0.95, 0.95; // Slow dynamics exponent gain
      alpha      << 0.75, 0.75, 0.75, 0.70, 0.70, 0.60; // Slow dynamics exponent gain
      //beta       << 1.60, 1.60, 1.60, 1.60, 1.60, 1.60; // Fast dynamics exponent gain

      //beta       << 1.75, 1.75, 1.75, 1.75, 1.75, 1.75; // Fast dynamics exponent gain
      beta       << 1.75, 1.75, 1.75, 1.50, 1.50, 1.50; // Fast dynamics exponent gain
      
      // Init algorithmic class
      for (int i=0; i<6; i++){
        //FXTTD new_fxttd(dt, z_1(i), sim_pose(i));
        FXTTD new_fxttd(dt, z_1(i));
	new_fxttd.setGains(epsilon1(i), epsilon2(i), epsilon3(i), alpha(i), beta(i));
        FXTTDs.push_back(new_fxttd);
      }

    }

    void sim_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      sim_pose_msg = *msg;

      sim_pose <<  sim_pose_msg.pose.pose.position.x,
                   sim_pose_msg.pose.pose.position.y,
                   sim_pose_msg.pose.pose.position.z;

      sim_quat.w() =  sim_pose_msg.pose.pose.orientation.w;
      sim_quat.x() =  sim_pose_msg.pose.pose.orientation.x;
      sim_quat.y() =  sim_pose_msg.pose.pose.orientation.y;
      sim_quat.z() =  sim_pose_msg.pose.pose.orientation.z;

      // Rotate velocity to world frame, because it comes from odom plugin
      vel_body.w() = 0.0;
      vel_body.x() = sim_pose_msg.twist.twist.linear.x;
      vel_body.y() = sim_pose_msg.twist.twist.linear.y;
      vel_body.z() = sim_pose_msg.twist.twist.linear.z;

      //vel_world = sim_quat * vel_body * sim_quat.conjugate();

      //sim_vel << vel_world.x(),
      //		 vel_world.y(),
      //		 vel_world.z();
      
      sim_vel << sim_pose_msg.twist.twist.linear.x, 
	         sim_pose_msg.twist.twist.linear.y,
		 sim_pose_msg.twist.twist.linear.z;
       

      sim_omega << sim_pose_msg.twist.twist.angular.x, 
		   sim_pose_msg.twist.twist.angular.y,
		   sim_pose_msg.twist.twist.angular.z;
      

    }

    void control_callback(){
      // Compute linear error
      e_diff(0) = sim_pose(0) - z_1(0);
      e_diff(1) = sim_pose(1) - z_1(1);
      e_diff(2) = sim_pose(2) - z_1(2);

      // Compute angular error, would use normal difference, but to avoid special handling with wrapparound, compute using q_e approach
      q_e = q_hat.conjugate() * sim_quat; 
      q_e.normalize();

      //QLM
      q_e_vec = q_e.vec();

      if (q_e_vec.norm() < 0.000001){
        q_e_vec << 0.0, 0.0, 0.0;
      } else { 
        q_e_vec = 2.0*(q_e_vec.normalized() * acos(q_e.w()));
      }
      
      e_diff(3) = q_e_vec(0);
      e_diff(4) = q_e_vec(1);
      e_diff(5) = q_e_vec(2);

      // Run FXTTD for each DOF
      for (int i = 0; i < 6; i++){
	//FXTTDs[i].setOdom(sim_pose(i));
	FXTTDs[i].setError(e_diff(i));
	FXTTDs[i].compute();
	z_1(i) = FXTTDs[i].get_z1();
	z_2(i) = FXTTDs[i].get_z2();
	z_3(i) = FXTTDs[i].get_z3();
      }

      // Reconvert eta_hat to quaternion
      q_e_vec << z_1(3), z_1(4), z_1(5);

      if (q_e_vec.norm() < 0.000001){
	q_hat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      } else {
        q_hat.w() = cos(q_e_vec.norm()/2.0);
        q_e_vec   = (q_e_vec.normalized()) * sin(q_e_vec.norm()/2.0);
	q_hat.x() = q_e_vec(0);
	q_hat.y() = q_e_vec(1);
	q_hat.z() = q_e_vec(2);
      }

      // Normalize q_hat
      q_hat.normalize();

      // Publish diff pose
      diff_pose_msg.header.stamp            = this->get_clock()->now();
      diff_pose_msg.header.frame_id         = this->get_parameter("tf_namespace").as_string(); 
      diff_pose_msg.child_frame_id          = this->get_parameter("tf_namespace").as_string() + "/diff"; 
      diff_pose_msg.pose.pose.position.x    = z_1(0);
      diff_pose_msg.pose.pose.position.y    = z_1(1);
      diff_pose_msg.pose.pose.position.z    = z_1(2);
      diff_pose_msg.pose.pose.orientation.w = q_hat.w();
      diff_pose_msg.pose.pose.orientation.x = q_hat.x();
      diff_pose_msg.pose.pose.orientation.y = q_hat.y();
      diff_pose_msg.pose.pose.orientation.z = q_hat.z();
      diff_pose_msg.twist.twist.linear.x    = z_2(0);
      diff_pose_msg.twist.twist.linear.y    = z_2(1);
      diff_pose_msg.twist.twist.linear.z    = z_2(2);
      diff_pose_msg.twist.twist.angular.x   = z_2(3);
      diff_pose_msg.twist.twist.angular.y   = z_2(4);
      diff_pose_msg.twist.twist.angular.z   = z_2(5);
      diff_pose_publisher->publish(diff_pose_msg);

      nav_msgs::msg::Odometry error_msg;
      error_msg.header.stamp = this->get_clock()->now();
      error_msg.header.frame_id = this->get_parameter("tf_namespace").as_string();
      error_msg.pose.pose.position.x    = sim_pose(0) - z_1(0);
      error_msg.pose.pose.position.y    = sim_pose(1) - z_1(1);
      error_msg.pose.pose.position.z    = sim_pose(2) - z_1(2);
      error_msg.pose.pose.orientation.w = q_e.w();
      error_msg.pose.pose.orientation.x = q_e.x();
      error_msg.pose.pose.orientation.y = q_e.y();
      error_msg.pose.pose.orientation.z = q_e.z();
      error_msg.twist.twist.linear.x    = sim_vel(0)   - z_2(0);
      error_msg.twist.twist.linear.y    = sim_vel(1)   - z_2(1);
      error_msg.twist.twist.linear.z    = sim_vel(2)   - z_2(2);
      error_msg.twist.twist.angular.x   = sim_omega(0) - z_2(3);
      error_msg.twist.twist.angular.y   = sim_omega(1) - z_2(4);
      error_msg.twist.twist.angular.z   = sim_omega(2) - z_2(5);
      diff_error_publisher->publish(error_msg);

      // Tf
      sim_tf.header.stamp = this->get_clock()->now();
      sim_tf.header.frame_id = "world";
      sim_tf.child_frame_id = this->get_parameter("tf_namespace").as_string(); 
      sim_tf.transform.translation.x = z_1(0);
      sim_tf.transform.translation.y = z_1(1);
      sim_tf.transform.translation.z = z_1(2);
      sim_tf.transform.rotation.w    = q_hat.w();
      sim_tf.transform.rotation.x    = q_hat.x();
      sim_tf.transform.rotation.y    = q_hat.y();
      sim_tf.transform.rotation.z    = q_hat.z();
      tf_broadcaster->sendTransform(sim_tf);
    }

  private:

    nav_msgs::msg::Odometry              sim_pose_msg;
    nav_msgs::msg::Odometry              diff_pose_msg;
    geometry_msgs::msg::AccelStamped     diff_accel_msg;
    geometry_msgs::msg::TransformStamped sim_tf;

    // Precalculated
    Eigen::Vector3d    sim_pose;         // Simulated position
    Eigen::Vector3d    sim_vel;          // Simulated velocity
    Eigen::Vector3d    sim_omega;        // Simulated angular velocity
    Eigen::Quaterniond sim_quat;         // Simulated quaternion
    // Precalculated, helpers
    Eigen::Quaterniond vel_body;         // Velocity in body frame (from odom plugin)
    Eigen::Quaterniond vel_world;        // Velocity in world frame

    // Algorithmic
    std::vector<FXTTD> FXTTDs;           // Differentiators
    Eigen::VectorXd    e_diff;        // General differentiator error, n-dimensional, feed to class
    Eigen::VectorXd    z_1;           // First  degree diff estimate,  n-dimensional, extract from class
    Eigen::VectorXd    z_2;           // Second degree diff estimate,  n-dimensional, extract from class
    Eigen::VectorXd    z_3;           // Third  degree diff estimate,  n-dimensional, extract from class
    Eigen::Quaterniond q_hat;            // Quaternion estimate
    Eigen::Quaterniond q_e;              // Quaternion error
    Eigen::Vector3d    q_e_vec;          // Quaternion error axis angle representation
  
    // Gains 
    Eigen::VectorXd    epsilon1;      // First estimation gain,        n-dimensional, feed to class
    Eigen::VectorXd    epsilon2;      // Second estimation gain,       n-dimensional, feed to class
    Eigen::VectorXd    epsilon3;      // Third estimation gain,        n-dimensional, feed to class
    Eigen::VectorXd    alpha;         // Slow dynamics exponent gain,  n-dimensional, feed to class
    Eigen::VectorXd    beta;          // Fast dynamics exponent gain,  n-dimensional, feed to class


    //double dt = 0.05; // Control loop time step
    double dt = 0.01; // Control loop time step

    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_pose_subscriber;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    diff_pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    diff_error_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FXTTD_Node>());
  rclcpp::shutdown();
  return 0;
}
