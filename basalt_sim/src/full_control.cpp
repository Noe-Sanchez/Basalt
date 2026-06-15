#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
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

using namespace std::chrono_literals;

class EController : public rclcpp::Node{
  public:
    EController(): Node("control_node"){
      // Namespace for tf
      this->declare_parameter<std::string>("tf_namespace", "omni_1");

      // Subscribers;
      sim_pose_subscriber     = this->create_subscription<nav_msgs::msg::Odometry>("/model/omni_1/odometry",    10, std::bind(&EController::sim_pose_callback,     this, std::placeholders::_1));
      desired_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/control_1/reference/pose", 10, std::bind(&EController::desired_pose_callback, this, std::placeholders::_1));

      // Temporal bool for diff
      kill_switch_subscriber = this->create_subscription<std_msgs::msg::Bool>("/control_1/kill_switch", 10, std::bind(&EController::kill_switch_callback, this, std::placeholders::_1));

      // Publishers
      control_f_publisher = this->create_publisher<geometry_msgs::msg::Wrench>("/control_1/control_force", 10);
      motor_publisher     = this->create_publisher<actuator_msgs::msg::Actuators>("/omni_1/command/motor_speed", 10);
      error_publisher     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/control_1/error", 10);
      sigma_publisher     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/control_1/sigma", 10);
      K_publisher         = this->create_publisher<geometry_msgs::msg::Vector3>("/control_1/K_values", 10);
      tf_broadcaster      = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      //control_timer   = this->create_wall_timer(10ms, std::bind(&EController::control_callback, this));
      control_timer   = this->create_wall_timer(50ms, std::bind(&EController::control_callback, this));

      m = 2.0;

      // x500 values
      //Jxx = 0.021666;
      //Jyy = 0.021666;
      //Jzz = 0.040000;

      // Omni values
      Jxx = 0.085225;
      Jyy = 0.085225;
      Jzz = 0.085225;

      J << Jxx, 0,  0,
            0, Jyy, 0,
            0, 0, Jzz;

      //kT = 8.54858e-6; // X500 motor
      //kQ = kT*0.016; // Use of motor constant X500
      kT = 2.50972e-5; // Omni motor
      kQ = kT*0.06; // Use of motor constant Omni
      //l  = 0.25;
      l = 0.1732;

      kp_lin << 2.0, 2.0, 2.0;
      kd_lin << 5.0, 5.0, 5.0;
      ki_lin << 0.5, 0.5, 0.5;

      kmin   <<     4.0,  4.0,  0.5; 
      k1     <<     5.0,  5.0, 0.25;
      k2     <<     0.05, 0.05, 0.5; 
      lambda <<     1.5,  1.5,  1.0;
      mu     <<     0.5, 0.5, 0.1;
      sigma  <<     0.0,  0.0,  0.0;
      K      <<     0.0,  0.0,  0.0;
      K_dot  <<     0.0,  0.0,  0.0;
      K_dot_prev << 0.0,  0.0,  0.0;
      
      kp_ang << 90.0,  90.0, 90.0;
      kd_ang << 10.0,  10.0, 10.0;
      ki_ang << 10.0,  10.0, 10.0;

      e_lin         << 0.0, 0.0, 0.0;
      e_lin_prev    << 0.0, 0.0, 0.0;
      e_lin_int     << 0.0, 0.0, 0.0;
      e_dot_lin     << 0.0, 0.0, 0.0;
      sim_pos       << 0.0, 0.0, 0.0;
      desired_pos   << 0.0, 0.0, 2.0;
      sim_vel       << 0.0, 0.0, 0.0;
      desired_vel   << 0.0, 0.0, 0.0;
      sim_omega     << 0.0, 0.0, 0.0;
      desired_omega << 0.0, 0.0, 0.0;
      u_ang         << 0.0, 0.0, 0.0;
      fu            << 0.0, 0.0, 0.0;
      ft            << 0.0, 0.0, 1.0;
      uaux_lin      << 0.0, 0.0, 0.0;
      uaux_ang      << 0.0, 0.0, 0.0;
      g_vector      << 0.0, 0.0, m*9.81;
      e_ang         << 0.0, 0.0, 0.0;
      e_dot_ang     << 0.0, 0.0, 0.0;
      e_ang_prev    << 0.0, 0.0, 0.0;
      e_ang_int     << 0.0, 0.0, 0.0;
      feed_lin      << 0.0, 0.0, 0.0;
      feed_ang      << 0.0, 0.0, 0.0;

      flat_outputs.resize(6);
      motor_speeds.resize(8);

      flat_outputs  << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      motor_speeds  << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

      sim_quat     = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      desired_quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      qud          = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      qe           = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      vel_body     = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      vel_world    = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

      // Actuation matrix for 8 cube omni ( Brescianini 2016 )
      actuation.resize(6, 8);

      // Auxiliar matrices for composing actuation
      Eigen::MatrixXd P(3, 8);
      Eigen::MatrixXd X(3, 8);

      float a, b, c;
      a = 0.5 + (1/(sqrt(12)));
      b = 0.5 - (1/(sqrt(12)));
      c = 1/(sqrt(3));

      P << 1, -1,  1, -1,  1, -1,  1, -1,
	   1,  1, -1, -1,  1,  1, -1, -1,
	   1,  1,  1,  1, -1, -1, -1, -1;
      P = c * P;

      X << -a,  b, -b,  a,  a, -b,  b, -a,
	    b,  a, -a, -b, -b, -a,  a,  b,
	    c, -c, -c,  c,  c, -c, -c,  c;

      float spin_dir[8] = {1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0};

      // Compose by column
      for (int i = 0; i < 8; i++) {
	//actuation.col(i) << X.col(i), P.col(i).cross(X.col(i));
	Eigen::VectorXd col(6);
	Eigen::Vector3d v1, v2, v3;
	v1 = X.col(i);
	v2 = P.col(i)*l;
	//v3 = v2.cross(v1) + (spin_dir[i]*0.06 * v1);
	v3 = v2.cross(v1) + (spin_dir[i]*0.06 * v1);
	col << v1, v3;	
	actuation.col(i) = col;
      }
      // Actual actuation
      //actuation = actuation.transpose() * (actuation * actuation.transpose()).inverse();
      actuation = actuation.completeOrthogonalDecomposition().pseudoInverse();

      motor_speed.velocity.resize(8);
    }

    void kill_switch_callback(const std_msgs::msg::Bool::SharedPtr msg){
      kill_switch = msg->data;
      if (kill_switch) {
        std::cout << "Kill switch ON" << std::endl;
      } else {
        std::cout << "Kill switch OFF" << std::endl;
      }
    }

    void sim_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      sim_pose = *msg;

      sim_pos <<  sim_pose.pose.pose.position.x,
                  sim_pose.pose.pose.position.y,
                  sim_pose.pose.pose.position.z;

      sim_quat.w() =  sim_pose.pose.pose.orientation.w;
      sim_quat.x() =  sim_pose.pose.pose.orientation.x;
      sim_quat.y() =  sim_pose.pose.pose.orientation.y;
      sim_quat.z() =  sim_pose.pose.pose.orientation.z;

      // Rotate velocity to world frame, because it comes from odom plugin
      vel_body.w() = 0.0;
      vel_body.x() = sim_pose.twist.twist.linear.x;
      vel_body.y() = sim_pose.twist.twist.linear.y;
      vel_body.z() = sim_pose.twist.twist.linear.z;

      vel_world = sim_quat * vel_body * sim_quat.conjugate();

      sim_vel << vel_world.x(),
		 vel_world.y(),
		 vel_world.z();

      sim_omega << sim_pose.twist.twist.angular.x, 
		   sim_pose.twist.twist.angular.y,
		   sim_pose.twist.twist.angular.z;
      
      // tf
      sim_tf.header.stamp = this->get_clock()->now();
      sim_tf.header.frame_id = "world";
      //sim_tf.child_frame_id = "x500";
      sim_tf.child_frame_id = this->get_parameter("tf_namespace").as_string(); 
      sim_tf.transform.translation.x = sim_pos(0);
      sim_tf.transform.translation.y = sim_pos(1);
      sim_tf.transform.translation.z = sim_pos(2);
      sim_tf.transform.rotation.w = sim_quat.w();
      sim_tf.transform.rotation.x = sim_quat.x();
      sim_tf.transform.rotation.y = sim_quat.y();
      sim_tf.transform.rotation.z = sim_quat.z();
      tf_broadcaster->sendTransform(sim_tf);

    }


    void desired_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      standby = false;

      desired_pose = *msg;

      desired_pos << desired_pose.pose.pose.position.x,
                     desired_pose.pose.pose.position.y,
                     desired_pose.pose.pose.position.z;

      desired_vel << desired_pose.twist.twist.linear.x,
                     desired_pose.twist.twist.linear.y,
                     desired_pose.twist.twist.linear.z;

      desired_quat.w() = desired_pose.pose.pose.orientation.w;
      desired_quat.x() = desired_pose.pose.pose.orientation.x;
      desired_quat.y() = desired_pose.pose.pose.orientation.y;
      desired_quat.z() = desired_pose.pose.pose.orientation.z;

      desired_omega << desired_pose.twist.twist.angular.x,
		       desired_pose.twist.twist.angular.y,
		       desired_pose.twist.twist.angular.z;
    }

    void control_callback(){
      if (standby) {
	desired_pos << sim_pos(0), sim_pos(1), 2.0;
	desired_vel << 0.0, 0.0, 0.0;
      }

      // Compute errors
      e_lin     = desired_pos - sim_pos;
      e_dot_lin = desired_vel - sim_vel;

      // Integrate linear error
      //e_lin_int = e_lin_int + 0.5*(e_lin + e_lin_prev)*0.01;
      e_lin_int = e_lin_int + 0.5*(e_lin + e_lin_prev)*0.05;
      e_lin_prev = e_lin;

      //std::cout << "e_lin_int: " << e_lin_int.transpose() << std::endl;

      // Compute linear control (angular control needs fu)
      //uaux_lin = kp_lin.cwiseProduct(e_lin) + kd_lin.cwiseProduct(e_dot_lin) + ki_lin.cwiseProduct(e_lin_int); 

      // Kmin sliding mode control
      sigma = e_dot_lin + lambda.cwiseProduct(e_lin);
      std::cout << "sigma: " << sigma.transpose() << std::endl;
      //sigma = e_lin + lambda.cwiseProduct(e_dot_lin);

      // Compute K_dot
      for (int i = 0; i < 3; i++) {
	if (K(i) > kmin(i)) {
	  K_dot(i) = k1(i) * sign( abs(sigma(i)) - mu(i) );
	} else {
	  K_dot(i) = kmin(i);
	}
      }

      // Integrate K_dot
      K = K + 0.5*(K_dot + K_dot_prev)*0.05;
      K_dot_prev = K_dot;

      //K(1) = 2.0;
      //K(2) = 2.0;

      // Compute nominal control law
      for (int i = 0; i < 3; i++) {
	uaux_lin(i) = -K(i) * pow(abs(sigma(i)), 0.5) * sign(sigma(i)) - k2(i) * sigma(i);
	//uaux_lin(i) = -k1(i) * pow(abs(sigma(i)), 0.5) * sign(sigma(i)) - k2(i) * sigma(i);
      }

      // Compute total control law
      fu = m*(feed_lin - uaux_lin + lambda.cwiseProduct(e_dot_lin));
      //fu(2) = -fu(2);

      // Saturate forces as safety
      fu(0) = std::max(-m*2.5,  std::min(m*2.5, fu(0)));
      fu(1) = std::max(-m*2.5,  std::min(m*2.5, fu(1)));
      //fu(0) = 0;
      //fu(1) = 0;
      fu(2) = std::max(-m*5.0,  std::min(m*5.0, fu(2)));

      // Add gravity compensation
      fu = fu + g_vector;

      // Rotate, since we dont do Olivas code
      // Use eigen rotation shorthand alias
      fu = sim_quat.conjugate() * fu;
      
      /*
      // Olivas Tesis 2.51, get desired quaternion for thrust
      if ( abs(fu.normalized().dot(ft)) == 1.0 ) { 
        qud.w() = 1.0;
      } else {
	qud.w() = sqrt((1 + (fu.normalized()).dot(ft))/2.0);
      }
      if (fu.normalized().cross(ft).norm() < 0.0001) {
	qud.x() = 0.0;
	qud.y() = 0.0;
	qud.z() = 0.0;
      } else {
	Eigen::Vector3d axis = (((fu.normalized()).cross(ft)).normalized())*sqrt((1 - (fu.normalized()).dot(ft))/2.0);
        qud.x() = axis(0);	
	qud.y() = axis(1);
	qud.z() = axis(2);
      }
      qud = qud * desired_quat;

      qe = sim_quat.inverse() * qud;
      qe.normalize();
      */
      //qe = sim_quat.inverse() * desired_quat;
      //qe.normalize();

      Eigen::Vector4d sim_parts, desired_parts;
      sim_parts << sim_quat.w(), sim_quat.x(), sim_quat.y(), sim_quat.z();
      desired_parts << desired_quat.w(), desired_quat.x(), desired_quat.y(), desired_quat.z();
      if ( sim_parts.dot(desired_parts) < 0.0 ) {
	desired_quat.w() = -desired_quat.w();
	desired_quat.x() = -desired_quat.x();
	desired_quat.y() = -desired_quat.y();
	desired_quat.z() = -desired_quat.z();
      }
      qe = sim_quat.inverse() * desired_quat;
      qe.normalize();
      
      // Constraint to upper hemisphere
      /*if (qe.w() < 0.0) {
        qe.w() = -qe.w();
	qe.x() = -qe.x();
	qe.y() = -qe.y();
	qe.z() = -qe.z();
      }*/

      // Compute QLM of error quaternion
      /*double norm_qe = sqrt(qe.x()*qe.x() + qe.y()*qe.y() + qe.z()*qe.z());
      if (norm_qe < 0.0001) {
	e_ang << 0.0, 0.0, 0.0;
      } else {
	double acosaux = acos(std::clamp(qe.w(), -1.0, 1.0));
	e_ang << qe.x(), qe.y(), qe.z();
	e_ang = 2 * e_ang.normalized() * acosaux;
      }*/

      // Use vector part sign corrected control law
      e_ang << qe.x(), qe.y(), qe.z();
      e_ang = e_ang * sign(qe.w());

      //std::cout << "e_ang: " << e_ang.transpose() << std::endl;

      // Compute angular error derivative
      e_dot_ang = desired_omega - sim_omega;

      // Integrate angular error
      e_ang_int = e_ang_int + 0.5*(e_ang + e_ang_prev)*0.05;
      e_ang_prev = e_ang;

      // Compute angular control
      //uaux_ang = kp_ang.cwiseProduct(e_ang) + kd_ang.cwiseProduct(e_dot_ang);
      uaux_ang = kp_ang.cwiseProduct(e_ang) + kd_ang.cwiseProduct(e_dot_ang) + ki_ang.cwiseProduct(e_ang_int);

      // Olivas ICUAS23 (24)
      u_ang = J * (feed_ang + uaux_ang) + (sim_omega.cross(J * sim_omega));

      // Castañeda ICUAS17 (39)
      flat_outputs << fu(0), fu(1), fu(2), u_ang(0), u_ang(1), u_ang(2); 
      //flat_outputs << fu(0), fu(1), fu(2), 0.0, 0.0, 0.0;

      // Pseudo inverse for allocation
      motor_speeds = actuation*flat_outputs;
      /*
      // Min motor speeds
      motor_speeds(0) = std::max(0.0, motor_speeds(0));
      motor_speeds(1) = std::max(0.0, motor_speeds(1));
      motor_speeds(2) = std::max(0.0, motor_speeds(2));
      motor_speeds(3) = std::max(0.0, motor_speeds(3));
      motor_speeds = motor_speeds.cwiseSqrt();
      */

      std::cout << "Motor speeds (rad/s): " << motor_speeds.transpose() << std::endl;

      /*motor_speeds(0) = std::max(0.0, motor_speeds(0));
      motor_speeds(1) = std::max(0.0, motor_speeds(1));
      motor_speeds(2) = std::max(0.0, motor_speeds(2));
      motor_speeds(3) = std::max(0.0, motor_speeds(3));
      motor_speeds(4) = std::max(0.0, motor_speeds(4));
      motor_speeds(5) = std::max(0.0, motor_speeds(5));
      motor_speeds(6) = std::max(0.0, motor_speeds(6));
      motor_speeds(7) = std::max(0.0, motor_speeds(7));

      motor_speeds(0) = (1/kT) * motor_speeds(0);
      motor_speeds(1) = (1/kT) * motor_speeds(1);
      motor_speeds(2) = (1/kT) * motor_speeds(2);
      motor_speeds(3) = (1/kT) * motor_speeds(3);
      motor_speeds(4) = (1/kT) * motor_speeds(4);
      motor_speeds(5) = (1/kT) * motor_speeds(5);
      motor_speeds(6) = (1/kT) * motor_speeds(6);
      motor_speeds(7) = (1/kT) * motor_speeds(7);


      // Because of omni actuation, we can have negative motor speeds
      motor_speeds(0) = sqrt(motor_speeds(0));
      motor_speeds(1) = sqrt(motor_speeds(1));
      motor_speeds(2) = sqrt(motor_speeds(2));
      motor_speeds(3) = sqrt(motor_speeds(3));
      motor_speeds(4) = sqrt(motor_speeds(4));
      motor_speeds(5) = sqrt(motor_speeds(5));
      motor_speeds(6) = sqrt(motor_speeds(6));
      motor_speeds(7) = sqrt(motor_speeds(7));*/

      motor_speeds(0) = sign(motor_speeds(0)) * sqrt(fabs((1/kT) * motor_speeds(0)));  
      motor_speeds(1) = sign(motor_speeds(1)) * sqrt(fabs((1/kT) * motor_speeds(1)));
      motor_speeds(2) = sign(motor_speeds(2)) * sqrt(fabs((1/kT) * motor_speeds(2)));
      motor_speeds(3) = sign(motor_speeds(3)) * sqrt(fabs((1/kT) * motor_speeds(3)));
      motor_speeds(4) = sign(motor_speeds(4)) * sqrt(fabs((1/kT) * motor_speeds(4)));
      motor_speeds(5) = sign(motor_speeds(5)) * sqrt(fabs((1/kT) * motor_speeds(5)));
      motor_speeds(6) = sign(motor_speeds(6)) * sqrt(fabs((1/kT) * motor_speeds(6)));
      motor_speeds(7) = sign(motor_speeds(7)) * sqrt(fabs((1/kT) * motor_speeds(7)));

      // Publish motor speeds
      motor_speed.header.stamp = this->get_clock()->now();
      motor_speed.header.frame_id = "sim/motor_speed";
      if (kill_switch) {
        motor_speed.velocity[0] = 0.0;
	motor_speed.velocity[1] = 0.0;
	motor_speed.velocity[2] = 0.0;
	motor_speed.velocity[3] = 0.0;
	motor_speed.velocity[4] = 0.0;
	motor_speed.velocity[5] = 0.0;
	motor_speed.velocity[6] = 0.0;
	motor_speed.velocity[7] = 0.0;
      } else {
        /*motor_speed.velocity[0] = std::clamp(motor_speeds(0), 0.0, 4000.0); 
        motor_speed.velocity[1] = std::clamp(motor_speeds(1), 0.0, 4000.0);
        motor_speed.velocity[2] = std::clamp(motor_speeds(2), 0.0, 4000.0);
        motor_speed.velocity[3] = std::clamp(motor_speeds(3), 0.0, 4000.0);
        motor_speed.velocity[4] = std::clamp(motor_speeds(4), 0.0, 4000.0);
	motor_speed.velocity[5] = std::clamp(motor_speeds(5), 0.0, 4000.0);
	motor_speed.velocity[6] = std::clamp(motor_speeds(6), 0.0, 4000.0);
	motor_speed.velocity[7] = std::clamp(motor_speeds(7), 0.0, 4000.0);*/
	motor_speed.velocity[0] = std::clamp(motor_speeds(0), -1000.0, 1000.0);
	motor_speed.velocity[1] = std::clamp(motor_speeds(1), -1000.0, 1000.0);
	motor_speed.velocity[2] = std::clamp(motor_speeds(2), -1000.0, 1000.0);
	motor_speed.velocity[3] = std::clamp(motor_speeds(3), -1000.0, 1000.0);
	motor_speed.velocity[4] = std::clamp(motor_speeds(4), -1000.0, 1000.0);
	motor_speed.velocity[5] = std::clamp(motor_speeds(5), -1000.0, 1000.0);
	motor_speed.velocity[6] = std::clamp(motor_speeds(6), -1000.0, 1000.0);
	motor_speed.velocity[7] = std::clamp(motor_speeds(7), -1000.0, 1000.0);
        //motor_speed.velocity[0] = std::clamp(motor_speeds(0), 400.0, 2000.0); 
        //motor_speed.velocity[1] = std::clamp(motor_speeds(1), 400.0, 2000.0);
        //motor_speed.velocity[2] = std::clamp(motor_speeds(2), 400.0, 2000.0);
        //motor_speed.velocity[3] = std::clamp(motor_speeds(3), 400.0, 2000.0); 
      }
      motor_publisher->publish(motor_speed);

      // Publish error
      error_msg.header.stamp = this->get_clock()->now();
      error_msg.header.frame_id = "world";
      error_msg.pose.position.x = e_lin(0);
      error_msg.pose.position.y = e_lin(1);
      error_msg.pose.position.z = e_lin(2);
      error_msg.pose.orientation.w = qe.w(); 
      error_msg.pose.orientation.x = qe.x();
      error_msg.pose.orientation.y = qe.y();
      error_msg.pose.orientation.z = qe.z();
      error_publisher->publish(error_msg);
      
      error_msg.pose.position.x = sigma(0); 
      error_msg.pose.position.y = sigma(1);
      error_msg.pose.position.z = sigma(2);
      error_msg.pose.orientation.w = 0.0;
      error_msg.pose.orientation.x = e_ang(0);
      error_msg.pose.orientation.y = e_ang(1);
      error_msg.pose.orientation.z = e_ang(2);
      sigma_publisher->publish(error_msg);

      // Publish control force for visualization
      control_f.force.x  = fu(0);
      control_f.force.y  = fu(1);
      control_f.force.z  = fu(2) - g_vector(2); // Remove gravity for visualization
      control_f.torque.x = u_ang(0);
      control_f.torque.y = u_ang(1);
      control_f.torque.z = u_ang(2);
      control_f_publisher->publish(control_f);

      // Publish K values
      geometry_msgs::msg::Vector3 K_msg;
      K_msg.x = K(0);
      K_msg.y = K(1);
      K_msg.z = K(2);
      K_publisher->publish(K_msg);
    }

  private:

    nav_msgs::msg::Odometry sim_pose;
    nav_msgs::msg::Odometry desired_pose;
    geometry_msgs::msg::Wrench feedforward;
    geometry_msgs::msg::Wrench control_f;
    geometry_msgs::msg::TransformStamped sim_tf;
    actuator_msgs::msg::Actuators motor_speed;
    geometry_msgs::msg::PoseStamped error_msg;
    
    bool standby = true;

    Eigen::Matrix3d    J;             // Inertia tensor, kg m^2
    Eigen::Vector3d    e_lin;         // Linear error
    Eigen::Vector3d    e_lin_prev;    // Linear error previous
    Eigen::Vector3d    e_lin_int;     // Linear error integral
    Eigen::Vector3d    e_dot_lin;     // Linear error derivative
    Eigen::Vector3d    e_ang;         // Angular error
    Eigen::Vector3d    e_ang_prev;    // Angular error previous
    Eigen::Vector3d    e_ang_int;     // Angular error integral
    Eigen::Vector3d    e_dot_ang;     // Angular error derivative
    Eigen::Vector3d    sim_pos;       // Simulated position
    Eigen::Vector3d    desired_pos;   // Desired position
    Eigen::Vector3d    sim_vel;       // Simulated velocity
    Eigen::Vector3d    desired_vel;   // Desired velocity
    Eigen::Vector3d    sim_omega;     // Simulated angular velocity
    Eigen::Vector3d    desired_omega; // Desired angular velocity
    Eigen::Vector3d    u_ang;         // Angular control output
    Eigen::Vector3d    uaux_lin;      // Linear auxiliary control output
    Eigen::Vector3d    uaux_ang;      // Angular auxiliary control output
    Eigen::Vector3d    fu;            // Desired forces vector
    Eigen::Vector3d    ft;            // Thrust vector
    Eigen::Vector3d    kp_lin;        // Linear p gains
    Eigen::Vector3d    kd_lin;        // Linear d gains
    Eigen::Vector3d    ki_lin;        // Linear i gains
    Eigen::Vector3d    kp_ang;        // Angular p gains
    Eigen::Vector3d    kd_ang;        // Angular d gains
    Eigen::Vector3d    ki_ang;        // Angular i gains
 
    Eigen::Vector3d    kmin;          // Kmin sliding mode gain
    Eigen::Vector3d    k1;            // K1 sliding mode gain
    Eigen::Vector3d    k2;            // K2 sliding mode gain
    Eigen::Vector3d    lambda;        // Lambda sliding mode parameter
    Eigen::Vector3d    mu;            // Mu sliding mode parameter
    Eigen::Vector3d    sigma;         // Sigma sliding mode parameter
    Eigen::Vector3d    K;             // Adaptive gain
    Eigen::Vector3d    K_dot;         // Adaptive gain derivative
    Eigen::Vector3d    K_dot_prev;    // Adaptive gain derivative previous

    Eigen::Vector3d    g_vector;      // Gravity vector
    Eigen::Vector3d    feed_lin;      // Feedforward linear forces
    Eigen::Vector3d    feed_ang;      // Feedforward angular torques
    Eigen::Quaterniond sim_quat;     // Simulated quaternion
    Eigen::Quaterniond desired_quat; // Desired quaternion (only for yaw input)
    Eigen::Quaterniond qud;          // Internal quaternion for logarithmic mapping
    Eigen::Quaterniond qe;           // Quaternion error for logarithmic mapping
    Eigen::Quaterniond vel_body;     // Velocity in body frame (from odom plugin)
    Eigen::Quaterniond vel_world;    // Velocity in world frame
    Eigen::MatrixXd    actuation;    // Quadrotor actuation matrix
    Eigen::VectorXd    flat_outputs; // Overall flat outputs
    Eigen::VectorXd    motor_speeds; // Motor speeds for publishing

    float m;           // Mass, kg
    float Jxx;         // X-axis inertia
    float Jyy;         // Y-axis inertia
    float Jzz;         // Z-axis inertia
    float kT;          // Thrust coefficient
    float kQ;          // Torque coefficient
    float l;           // Rotor arm length, m

    bool kill_switch = false; // Temporal kill switch for safety

    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sim_pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      desired_pose_subscriber;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          kill_switch_subscriber;
    
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr      control_f_publisher;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr   motor_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sigma_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr     K_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EController>());
  rclcpp::shutdown();
  return 0;
}
