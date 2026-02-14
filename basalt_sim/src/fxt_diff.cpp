#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench.hpp"

double sign(double x){
  if (x > 0) {
    return 1.0;
  } else if (x < 0) {
    return -1.0;
  } else {
    return 0.0;
  }
}

Eigen::Vector3d sig3(Eigen::Vector3d v, double exp){
  Eigen::Vector3d result;
  
  result(0) = sign(v(0)) * pow(fabs(v(0)), exp);
  result(1) = sign(v(1)) * pow(fabs(v(1)), exp);
  result(2) = sign(v(2)) * pow(fabs(v(2)), exp);
  
  return result;
}

Eigen::Matrix3d gamma_matrix(Eigen::Quaterniond q){
  Eigen::Matrix3d gamma;

  gamma << q.w(), -q.z(),  q.y(),
	   q.z(),  q.w(), -q.x(),
	  -q.y(),  q.x(),  q.w();
	
  gamma = 0.5*gamma;

  return gamma;
}

// Quaternion multiplication by scalar
Eigen::Quaterniond qbys(double a, Eigen::Quaterniond q){
  Eigen::Quaterniond result;

  result.w() = a * q.w();
  result.x() = a * q.x();
  result.y() = a * q.y();
  result.z() = a * q.z();

  return result;
}

using namespace std::chrono_literals;

class FullDiff : public rclcpp::Node{
  public:
    FullDiff(): Node("fxt_diff_node"){
      // Namespace for tf
      this->declare_parameter<std::string>("tf_namespace", "x500_1");

      // Subscribers
      sim_pose_subscriber     = this->create_subscription<nav_msgs::msg::Odometry>("/model/x500_1/odometry",    10, std::bind(&FullDiff::sim_pose_callback,     this, std::placeholders::_1));

      // Publishers
      diff_pose_publisher      = this->create_publisher<nav_msgs::msg::Odometry>("/control_1/diff/odom", 10);
      extended_state_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/control_1/diff/extended_state", 10);
      diff_accel_publisher     = this->create_publisher<geometry_msgs::msg::AccelStamped>("/control_1/diff/accel", 10);
      tf_broadcaster           = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      control_timer   = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&FullDiff::control_callback, this));

      sim_quat            = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      vel_body            = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      vel_world           = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      diff_q_hat          = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      diff_q_hat_dot      = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      diff_q_hat_dot_prev = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      diff_q_tilde        = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      diff_q_helper       = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

      sim_pose              = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_vel               = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_omega             = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_omega_prev        = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_accel             = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_q_tilde_v        = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_omega_hat        = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_omega_hat_dot    = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_ext_hat          = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_ext_hat_dot      = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_ext_hat_dot_prev = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_q_helper_v       = Eigen::Vector3d(0.0, 0.0, 0.0);
      diff_gamma_q_inv      = Eigen::Matrix3d::Zero();

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

      vel_world = sim_quat * vel_body * sim_quat.conjugate();

      sim_vel << vel_world.x(),
		 vel_world.y(),
		 vel_world.z();

      sim_omega << sim_pose_msg.twist.twist.angular.x, 
		   sim_pose_msg.twist.twist.angular.y,
		   sim_pose_msg.twist.twist.angular.z;

      // Finite differencing for acceleration (dirty)
      sim_accel = (sim_omega - sim_omega_prev) / dt;
      sim_omega_prev = sim_omega;
      
      // tf
      sim_tf.header.stamp = this->get_clock()->now();
      sim_tf.header.frame_id = "world";
      sim_tf.child_frame_id = this->get_parameter("tf_namespace").as_string(); 
      sim_tf.transform.translation.x = sim_pose(0);
      sim_tf.transform.translation.y = sim_pose(1);
      sim_tf.transform.translation.z = sim_pose(2);
      sim_tf.transform.rotation.w = sim_quat.w();
      sim_tf.transform.rotation.x = sim_quat.x();
      sim_tf.transform.rotation.y = sim_quat.y();
      sim_tf.transform.rotation.z = sim_quat.z();
      tf_broadcaster->sendTransform(sim_tf);

    }

    void control_callback(){
      // Compute q_tilde
      diff_q_tilde = diff_q_hat.conjugate() * sim_quat; 

      // Get qv and gamma matrix
      diff_q_tilde_v = Eigen::Vector3d(diff_q_tilde.x(), diff_q_tilde.y(), diff_q_tilde.z());
      //diff_gamma_q_inv = -1*(gamma_matrix(diff_q_tilde).transpose()); // Transpose is ok, since gamma is skew-symmetric
      diff_gamma_q_inv = gamma_matrix(diff_q_tilde).inverse();

      // Extended state first
      diff_ext_hat_dot = kappa3*diff_gamma_q_inv * sig3(diff_q_tilde_v, alpha) + epsilon3*diff_gamma_q_inv * sig3(diff_q_tilde_v, beta);

      // Trapezoidal integral of ext_hat_dot
      diff_ext_hat += 0.5 * (diff_ext_hat_dot + diff_ext_hat_dot_prev) * dt;
      diff_ext_hat_dot_prev = diff_ext_hat_dot;

      // Angular velocity
      diff_omega_hat_dot = diff_ext_hat + kappa2*diff_gamma_q_inv * sig3(diff_q_tilde_v, (alpha+1)/2) + epsilon2*diff_gamma_q_inv * sig3(diff_q_tilde_v, (beta+1)/2); 

      // Trapezoidal integral of omega_hat_dot
      diff_omega_hat += 0.5 * (diff_omega_hat_dot + diff_omega_hat_dot_prev) * dt;
      diff_omega_hat_dot_prev = diff_omega_hat_dot;

      // Finally quaternion
      diff_q_helper_v = diff_omega_hat + kappa1*diff_gamma_q_inv * sig3(diff_q_tilde_v, (alpha+2)/3) + epsilon1*diff_gamma_q_inv * sig3(diff_q_tilde_v, (beta+2)/3);
      diff_q_helper.w() = 0.0;
      diff_q_helper.x() = diff_q_helper_v(0);
      diff_q_helper.y() = diff_q_helper_v(1);
      diff_q_helper.z() = diff_q_helper_v(2);

      diff_q_hat_dot = qbys(0.5, diff_q_hat) * diff_q_tilde * diff_q_helper * diff_q_tilde.conjugate();

      // Trapezoidal integral of q_hat_dot
      diff_q_hat.w() += 0.5 * (diff_q_hat_dot.w() + diff_q_hat_dot_prev.w()) * dt;
      diff_q_hat.x() += 0.5 * (diff_q_hat_dot.x() + diff_q_hat_dot_prev.x()) * dt;
      diff_q_hat.y() += 0.5 * (diff_q_hat_dot.y() + diff_q_hat_dot_prev.y()) * dt;
      diff_q_hat.z() += 0.5 * (diff_q_hat_dot.z() + diff_q_hat_dot_prev.z()) * dt;
      diff_q_hat.normalize();
      diff_q_hat_dot_prev = diff_q_hat_dot;
       
      // Publish to terminal (for now)
      //std::cout << "Estimated quaternion: "       << diff_q_hat.w()    << ", " << diff_q_hat.x()    << ", " << diff_q_hat.y()    << ", " << diff_q_hat.z() << std::endl;
      //std::cout << "Estimated angular velocity: " << diff_omega_hat(0) << ", " << diff_omega_hat(1) << ", " << diff_omega_hat(2) << std::endl;
      //std::cout << "Estimated extended state: "   << diff_ext_hat(0)   << ", " << diff_ext_hat(1)   << ", " << diff_ext_hat(2)   << std::endl;
      
      // Print errors and publish tf
      sim_tf.header.stamp    = this->get_clock()->now();
      sim_tf.header.frame_id = "world";
      sim_tf.child_frame_id  = "diff1";
      sim_tf.transform.translation.x = sim_pose(0);
      sim_tf.transform.translation.y = sim_pose(1);
      sim_tf.transform.translation.z = sim_pose(2);
      sim_tf.transform.rotation.w = diff_q_hat.w();
      sim_tf.transform.rotation.x = diff_q_hat.x();
      sim_tf.transform.rotation.y = diff_q_hat.y();
      sim_tf.transform.rotation.z = diff_q_hat.z();
      tf_broadcaster->sendTransform(sim_tf);

      diff_pose_msg.header.stamp = this->get_clock()->now();
      diff_pose_msg.header.frame_id = "world";
      diff_pose_msg.pose.pose.position.x = sim_pose(0);
      diff_pose_msg.pose.pose.position.y = sim_pose(1);
      diff_pose_msg.pose.pose.position.z = sim_pose(2);
      diff_pose_msg.pose.pose.orientation.w = diff_q_hat.w();
      diff_pose_msg.pose.pose.orientation.x = diff_q_hat.x();
      diff_pose_msg.pose.pose.orientation.y = diff_q_hat.y();
      diff_pose_msg.pose.pose.orientation.z = diff_q_hat.z();
      diff_pose_msg.twist.twist.angular.x = diff_omega_hat(0);
      diff_pose_msg.twist.twist.angular.y = diff_omega_hat(1);
      diff_pose_msg.twist.twist.angular.z = diff_omega_hat(2);
      diff_pose_publisher->publish(diff_pose_msg);

      diff_accel_msg.header.stamp = this->get_clock()->now();
      diff_accel_msg.header.frame_id = "world";
      diff_accel_msg.accel.linear.x = sim_accel(0); 
      diff_accel_msg.accel.linear.y = sim_accel(1);
      diff_accel_msg.accel.linear.z = sim_accel(2);
      diff_accel_msg.accel.angular.x = diff_ext_hat(0);
      diff_accel_msg.accel.angular.y = diff_ext_hat(1);
      diff_accel_msg.accel.angular.z = diff_ext_hat(2);

      Eigen::Quaterniond q_error = diff_q_hat.conjugate() * sim_quat;
      Eigen::Vector3d    vel_error = sim_omega - diff_omega_hat;
      Eigen::Vector3d    ext_error = sim_accel - diff_ext_hat;
      std::cout << "Quaternion error: " << q_error.w() << ", " << q_error.x() << ", " << q_error.y() << ", " << q_error.z() << std::endl;
      std::cout << "Angular velocity error: " << vel_error(0) << ", " << vel_error(1) << ", " << vel_error(2) << std::endl;
      std::cout << "Extended state error: "   << ext_error(0) << ", " << ext_error(1) << ", " << ext_error(2) << std::endl;

    }

  private:

    nav_msgs::msg::Odometry              sim_pose_msg;
    nav_msgs::msg::Odometry              diff_pose_msg;
    geometry_msgs::msg::AccelStamped     diff_accel_msg;
    geometry_msgs::msg::TransformStamped sim_tf;

    // Precalculated
    Eigen::Vector3d    sim_pose;       // Simulated position
    Eigen::Vector3d    sim_vel;        // Simulated velocity
    Eigen::Vector3d    sim_omega;      // Simulated angular velocity
    Eigen::Vector3d    sim_omega_prev; // Simulated angular velocity previous
    Eigen::Vector3d    sim_accel;      // Simulated acceleration (dirty, from finite differencing)
    Eigen::Quaterniond sim_quat;       // Simulated quaternion
    Eigen::Quaterniond vel_body;       // Velocity in body frame (from odom plugin)
    Eigen::Quaterniond vel_world;      // Velocity in world frame

    // Algorithmic
    Eigen::Quaterniond diff_q_hat;              // Estimated quaternion
    Eigen::Quaterniond diff_q_hat_dot;          // Estimated angular velocity (quaternion)
    Eigen::Quaterniond diff_q_hat_dot_prev;     // Estimated angular velocity (quaternion) previous
    Eigen::Quaterniond diff_q_tilde;            // Estimation error quaternion
    Eigen::Quaterniond diff_q_helper;           // Helper quaternion for calculations (R4 to R3)
    Eigen::Vector3d    diff_q_helper_v;         // Helper vector for calculations (R4 to R3)
    Eigen::Vector3d    diff_q_tilde_v;          // Estimation error quaternion vector part
    Eigen::Vector3d    diff_omega_hat;          // Estimated angular velocity (rates)
    Eigen::Vector3d    diff_omega_hat_dot;      // Estimated angular acceleration
    Eigen::Vector3d    diff_omega_hat_dot_prev; // Estimated angular acceleration previous
    Eigen::Vector3d    diff_ext_hat;            // Estimated extended state
    Eigen::Vector3d    diff_ext_hat_dot;        // Estimated extended state derivative
    Eigen::Vector3d    diff_ext_hat_dot_prev;   // Estimated extended state derivative previous
    Eigen::Matrix3d    diff_gamma_q_inv;        // Quaternion operator matrix inverse (for q_tilde)
    
    // Gains (need to satisfy hurwitz)
    double kappa1   = 1;
    double kappa2   = 1;
    double kappa3   = 1;
    double epsilon1 = 1;
    double epsilon2 = 1;
    double epsilon3 = 1;
    double epsilon4 = 1;
    double alpha    = 0.95;
    double beta     = 1.05;

    double dt       = 0.05; // Control loop time step

    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sim_pose_subscriber;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          diff_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  extended_state_publisher;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr diff_accel_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FullDiff>());
  rclcpp::shutdown();
  return 0;
}
