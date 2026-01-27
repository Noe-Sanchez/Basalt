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

// Algorithmic class for pttd, since operations are mostly element-wise
// Variables and methods for 1 DOF independent differentiation
class PTTD {
  public:
    PTTD(){}; // Default constructor, do nothing
    PTTD(double _dt=0.05, double _z_1=0.0, double _x_1=0.0, double _T_c=1.0){ 
      // Init algorithmic
      dt               =  _dt;
      e_diff           =  0.0;
      x_1              = _x_1; 
      z_1              = _z_1; 
      z_1_dot          =  0.0;
      z_1_dot_previous =  0.0;
      z_2              =  0.0;
      z_2_dot          =  0.0;
      z_2_dot_previous =  0.0;
      L                =  1.0;
      L_dot            =  0.0;	
      L_dot_previous   =  0.0;
	
      // Init gains
      T_c              = _T_c; // Convergence time
      p                =  1.5; // Convergence parameter
      k1               =  1.0; // Adaptive parameter 1
      k2               =  1.0; // Adaptive parameter 2
    }
    // Setters
    void setGains(double _p, double _k1, double _k2){
      p  = _p;
      k1 = _k1;
      k2 = _k2;
    }
    void setOdom(double _x_1){
      x_1 = _x_1; 
    }
    // Getters
    double get_z1(){
      return z_1;
    }
    double get_z2(){
      return z_2;
    }
    // Getters, plotting
    double get_L(){
      return L;
    }

    // Main algo, extract before recompute, run manually on correct dt
    void compute(){
      // Compute error
      e_diff = x_1 - z_1;

      // Adaptive gain dynamics
      L_dot = (k1 * (1.0/(T_c*(p-1.0))) * sqrt(fabs(e_diff)) * sqrt(L)) - (k2 * (L*L));
      // Trapezoidal integration 
      L = L + (L_dot + L_dot_previous)*0.5*dt;
      L_dot_previous = L_dot;

      // Sanity print for first term of L_dot
      std::cout << "First term L_dot: " << (k1 * (1.0/(T_c*(p-1.0))) * sqrt(fabs(e_diff)) * sqrt(L)) << std::endl;
      // Sanity print for second term of L_dot
      std::cout << "Second term L_dot: " << (k2 * (L * L)) << std::endl;
      // Check for potential problemmakers
      std::cout << "Sqrt of fabs(e_diff): " << sqrt(fabs(e_diff)) << std::endl;
      std::cout << "Sqrt of L: " << sqrt(L) << std::endl;
      std::cout << "Denominator T_c*(p-1.0): " << T_c*(p-1.0) << std::endl;
      std::cout << "L: " << L << std::endl;
      std::cout << "L_dot: " << L_dot << std::endl;
      
      // Second state
      z_2_dot = ( (2.0 * L * sign(e_diff)) / ( (T_c*T_c) * pow(p-1.0, 2.0) ) ) + ( sig(e_diff, (2.0*p)-1.0) / (2.0 * (T_c*T_c) * pow(p, 2.0)) );
      // Trapezoidal integration
      z_2 = z_2 + (z_2_dot + z_2_dot_previous)*0.5*dt;
      z_2_dot_previous = z_2_dot;
      
      // First state
      z_1_dot = z_2 + ( (sqrt(L) * sig(e_diff, 0.5)) / (T_c * (p-1.0)) ) + ( (2.0 * sig(e_diff, p)) / (T_c * (p)) ); 
      // Trapezoidal integration
      z_1 = z_1 + (z_1_dot + z_1_dot_previous)*0.5*dt;
      z_1_dot_previous = z_1_dot;

      // Variables are now exposed, extract z_1 and z_2 as estimates
      // Sanity print for L
      std::cout << "e_diff: " << e_diff << std::endl;
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
    double x_1;              // This DOF odom input
    double z_1;              // First degree diff estimate
    double z_1_dot;          // First degree diff estimate derivative
    double z_1_dot_previous; // First degree diff estimate derivative previous
    double z_2;              // Second degree diff estimate
    double z_2_dot;          // Second degree diff estimate derivative
    double z_2_dot_previous; // Second degree diff estimate derivative previous
    double L;                // Adaptive gain
    double L_dot;            // Adaptive gain dynamics
    double L_dot_previous;   // Adaptive gain dynamics previous
    
    // Gains
    double T_c;              // Convergence time
    double p;                // Convergence parameter
    double k1;               // Adaptive parameter 1
    double k2;               // Adaptive parameter 2

};

  

using namespace std::chrono_literals;

class PTTD_Node : public rclcpp::Node{
  public:
    PTTD_Node(): Node("pttd_node"){
      // Namespace for tf
      this->declare_parameter<std::string>("tf_namespace", "x500_1");

      // Subscribers
      sim_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/model/x500_1/odometry",    10, std::bind(&PTTD_Node::sim_pose_callback,     this, std::placeholders::_1));

      // Publishers
      diff_pose_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/control_1/diff/odom", 10);

      // Tf
      tf_broadcaster      = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      // Timer
      control_timer       = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&PTTD_Node::control_callback, this));

      // Init precalculated 
      sim_pose              = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_vel               = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_omega             = Eigen::Vector3d(0.0, 0.0, 0.0);
      sim_quat              = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      // Init pr0.0, 0.0, T_c(i));ecalc helpers
      vel_body              = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      vel_world             = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

      // Init algorithmic variables
      e_diff                = Eigen::Vector3d(0.0, 0.0, 0.0);
      z_1                   = Eigen::Vector3d(0.0, 0.0, 0.0);
      z_2                   = Eigen::Vector3d(0.0, 0.0, 0.0);
			
      // Init gains
      T_c                   = Eigen::Vector3d(2.0, 2.0, 2.0); // Convergence time
      p                     = Eigen::Vector3d(1.5, 1.5, 1.5); // Convergence parameter
      k1                    = Eigen::Vector3d(1.0, 1.0, 1.0); // Adaptive parameter 1
      k2                    = Eigen::Vector3d(0.5, 0.5, 0.5); // Adaptive parameter 2
      
      // Init algorithmic class
      for (int i=0; i<3; i++){
        PTTD new_pttd(dt, z_1(i), sim_pose(i), T_c(i)); 
        new_pttd.setGains(p(i), k1(i), k2(i));
        PTTDs.push_back(new_pttd);
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

      vel_world = sim_quat * vel_body * sim_quat.conjugate();

      sim_vel << vel_world.x(),
		 vel_world.y(),
		 vel_world.z();

      sim_omega << sim_pose_msg.twist.twist.angular.x, 
		   sim_pose_msg.twist.twist.angular.y,
		   sim_pose_msg.twist.twist.angular.z;
      
      // Tf
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
      // Compute error
      e_diff = sim_pose - z_1;

      // Run PTTD for each DOF
      for (int i=0; i<3; i++){
	PTTDs[i].setOdom(sim_pose(i));
	PTTDs[i].compute();
	z_1(i) = PTTDs[i].get_z1();
	z_2(i) = PTTDs[i].get_z2();
      }

      // Publish diff pose
      diff_pose_msg.header.stamp = this->get_clock()->now();
      diff_pose_msg.header.frame_id = this->get_parameter("tf_namespace").as_string(); 
      diff_pose_msg.child_frame_id  = this->get_parameter("tf_namespace").as_string() + "/diff"; 
      diff_pose_msg.pose.pose.position.x = z_1(0);
      diff_pose_msg.pose.pose.position.y = z_1(1);
      diff_pose_msg.pose.pose.position.z = z_1(2);
      diff_pose_msg.twist.twist.linear.x  = z_2(0);
      diff_pose_msg.twist.twist.linear.y  = z_2(1);
      diff_pose_msg.twist.twist.linear.z  = z_2(2);
      diff_pose_publisher->publish(diff_pose_msg);

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
    std::vector<PTTD>  PTTDs;            // Differentiators
    Eigen::Vector3d    e_diff;           // General differentiator error, n-dimensional, feed to class
    Eigen::Vector3d    z_1;              // First degree diff estimate, n-dimensional, extract from class
    Eigen::Vector3d    z_2;              // Second degree diff estimate, n-dimensional, extract from class
  
    // Gains 
    Eigen::Vector3d    T_c;              // Convergence time
    Eigen::Vector3d    p;                // Convergence parameter
    Eigen::Vector3d    k1;               // Adaptive parameter 1
    Eigen::Vector3d    k2;               // Adaptive parameter 2


    double dt = 0.05; // Control loop time step

    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_pose_subscriber;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    diff_pose_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PTTD_Node>());
  rclcpp::shutdown();
  return 0;
}
