#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/wrench.hpp"

#include <queue>

double sign(double x){
  if (x > 0) {
    return 1.0;
  } else if (x < 0) {
    return -1.0;
  } else {
    return 0.0;
  }
}

Eigen::Vector3d qlm(Eigen::Quaterniond q){
  Eigen::Vector3d v;
  double norm = sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
 
  if (norm < 0.0001) {
    v << q.x(), q.y(), q.z(); 
    v = v * 2.0; // Missing factor of 2 added here
  } else {
    double ang = acos(std::clamp(q.w(), -1.0, 1.0));
    v << q.x(), q.y(), q.z(); 
    v = 2.0 * v.normalized() * ang;
  }

  return v;
}

Eigen::Quaterniond qem(Eigen::Vector3d v){
  Eigen::Quaterniond q;
  double theta = v.norm();
  double half_theta = theta / 2.0;

  if (half_theta < 0.0001) {
    q.w() = 1.0;
    q.x() = v.x() / 2.0;
    q.y() = v.y() / 2.0;
    q.z() = v.z() / 2.0;
    q.normalize(); // Ensure strict unit length even on approximations
  } else {
    double s = sin(half_theta) / theta; // Divide by full theta
    double c = cos(half_theta);
    q.w() = c;
    q.x() = v.x() * s;
    q.y() = v.y() * s;
    q.z() = v.z() * s;
  }
  
  return q;
}

Eigen::Quaterniond qabs(Eigen::Quaterniond q){
  if (q.w() < 0) {
    return Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
  } else {
    return q;
  }
}


using namespace std::chrono_literals;

class LeaderInterp : public rclcpp::Node{
  public:
    LeaderInterp(): Node("leader_interp_node"){

      this->declare_parameter("segment_duration", 10.0);
      segment_duration = this->get_parameter("segment_duration").as_double();

      knot_subscriber     = this->create_subscription<nav_msgs::msg::Odometry>("reference/new_knot", 10, std::bind(&LeaderInterp::knot_callback, this, std::placeholders::_1));

      pause_subscriber    = this->create_subscription<std_msgs::msg::Bool>("reference/pause", 10, std::bind(&LeaderInterp::pause_callback, this, std::placeholders::_1));

      reference_publisher = this->create_publisher<nav_msgs::msg::Odometry>("reference/out", 10);
      viz_publisher       = this->create_publisher<visualization_msgs::msg::Marker>("reference/marker", 10);
      viz_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("reference/marker_array", 10);

      //std::chrono::duration<double> freq(period);
      control_timer = this->create_wall_timer(freq, std::bind(&LeaderInterp::control_callback, this));

      path_marker.header.frame_id = "world";
      path_marker.ns = "reference";
      path_marker.id = 0;
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.scale.x = 0.02;
      path_marker.color.a = 0.45;
      path_marker.color.r = 1.0;
      path_marker.color.g = 0.0;
      path_marker.color.b = 0.0;

      axis_marker.header.frame_id = "world";
      axis_marker.ns = "reference";
      axis_marker.type = visualization_msgs::msg::Marker::ARROW;
      axis_marker.action = visualization_msgs::msg::Marker::ADD;
      axis_marker.scale.x = 0.2;
      axis_marker.scale.y = 0.04;
      axis_marker.scale.z = 0.04;
      axis_marker.color.a = 1.0;
      //axis_marker.color.r = 0.0;
      //axis_marker.color.g = 1.0;
      //axis_marker.color.b = 0.0;
      
    }

    void pause_callback(const std_msgs::msg::Bool& msg){
      if (msg.data) {
	// Deregister timer
        std::cout << "Pausing trajectory execution." << std::endl;
        control_timer->cancel();
      } else {
	// Re-register timer
        control_timer = this->create_wall_timer(freq, std::bind(&LeaderInterp::control_callback, this));
	std::cout << "Resuming trajectory execution." << std::endl;
      }	
    }

    void knot_callback(const nav_msgs::msg::Odometry& msg){
      if (knots_received == 0) {
	r0 = msg;
      } else if (knots_received == 1) {
	r1 = msg;
       } else {
	reference_queue.push(msg);
       }
      knots_received++;
    }

    void control_callback(){
      // If there are less than 2 knots in the queue, we cannot interpolate
      if (knots_received < 2) {
	c = 0.0;
	return; 
      }

      if (c >= 1.0) {
	// Move to the next segment
	//marker.points.clear();
	r0 = r1;

	if (!reference_queue.empty()) {
	  r1 = reference_queue.front();
	  reference_queue.pop();
	} else {
	  //std::cout << "No more knots in the queue" << std::endl;
	  return;
	}
        
	std::cout << "Segment completed. Moving to the next segment r0: " << r0.pose.pose.position.x << ", " << r0.pose.pose.position.y << ", "
		<< r0.pose.pose.position.z << " r1: " << r1.pose.pose.position.x << ", " << r1.pose.pose.position.y << ", "
		<< r1.pose.pose.position.z << std::endl;
	c = 0.0;
      } else {
	// Print for debugging
	/*std::cout << "Interpolating between:\n";
	std::cout << "r0: " << r0.pose.pose.position.x << ", " << r0.pose.pose.position.y << ", "
		<< r0.pose.pose.position.z << std::endl;
	std::cout << "r1: " << r1.pose.pose.position.x << ", " << r1.pose.pose.position.y << ", "
		<< r1.pose.pose.position.z << std::endl;
	std::cout << "c: " << c << std::endl;
        std::cout << "Queue size: " << reference_queue.size() << std::endl;*/

	// Interpolate
	Eigen::Vector3d p0(r0.pose.pose.position.x, r0.pose.pose.position.y, r0.pose.pose.position.z);
	Eigen::Vector3d p1(r1.pose.pose.position.x, r1.pose.pose.position.y, r1.pose.pose.position.z);
	Eigen::Vector3d v0(r0.twist.twist.linear.x, r0.twist.twist.linear.y, r0.twist.twist.linear.z);
	Eigen::Vector3d v1(r1.twist.twist.linear.x, r1.twist.twist.linear.y, r1.twist.twist.linear.z);
	Eigen::Vector3d deltap = p1 - p0;

	double w1, w2, w3, W1, W2, W3;
	//double w1, w2, w3;

	w1 = 3*pow(c, 2) - 2*pow(c, 3);
	w2 =   pow(c, 3) - 2*pow(c, 2) + c;
	w3 =   pow(c, 3) -   pow(c, 2);

	W1 = 6*c - 6*pow(c, 2);
	W2 = 3*pow(c, 2) - 4*c + 1;
	W3 = 3*pow(c, 2) - 2*c;

        current_reference.pose.pose.position.x = p0.x() + w1*deltap.x() + w2*v0.x()*segment_duration + w3*v1.x()*segment_duration;
	current_reference.pose.pose.position.y = p0.y() + w1*deltap.y() + w2*v0.y()*segment_duration + w3*v1.y()*segment_duration;
	current_reference.pose.pose.position.z = p0.z() + w1*deltap.z() + w2*v0.z()*segment_duration + w3*v1.z()*segment_duration;

	current_reference.twist.twist.linear.x = W1*deltap.x()/segment_duration + W2*v0.x() + W3*v1.x(); 
	current_reference.twist.twist.linear.y = W1*deltap.y()/segment_duration + W2*v0.y() + W3*v1.y();
	current_reference.twist.twist.linear.z = W1*deltap.z()/segment_duration + W2*v0.z() + W3*v1.z();

	Eigen::Quaterniond q0(r0.pose.pose.orientation.w, r0.pose.pose.orientation.x, r0.pose.pose.orientation.y, r0.pose.pose.orientation.z);
	Eigen::Quaterniond q1(r1.pose.pose.orientation.w, r1.pose.pose.orientation.x, r1.pose.pose.orientation.y, r1.pose.pose.orientation.z);
	Eigen::Vector3d   vq0(r0.twist.twist.angular.x, r0.twist.twist.angular.y, r0.twist.twist.angular.z);
	Eigen::Vector3d   vq1(r1.twist.twist.angular.x, r1.twist.twist.angular.y, r1.twist.twist.angular.z);

	Eigen::Quaterniond current_q;
        Eigen::Vector3d delta_qv = qlm(qabs(q1 * q0.conjugate()));
	current_q = qem(w1*delta_qv + w2*vq0*segment_duration + w3*vq1*segment_duration) * q0;
	current_q.normalize();
	current_reference.pose.pose.orientation.w = current_q.w();
	current_reference.pose.pose.orientation.x = current_q.x();
	current_reference.pose.pose.orientation.y = current_q.y();
	current_reference.pose.pose.orientation.z = current_q.z();

	path_marker.points.push_back(current_reference.pose.pose.position);
	path_marker.header.stamp = this->get_clock()->now();
	viz_publisher->publish(path_marker);

	std::cout << c << std::endl;

	// Trunc c to avoid publishing too many arrows
	if (std::abs(fmod(std::trunc(c*1000), 100)) == 0) {
	  Eigen::Quaterniond red(current_reference.pose.pose.orientation.w, current_reference.pose.pose.orientation.x, current_reference.pose.pose.orientation.y, current_reference.pose.pose.orientation.z);
	  Eigen::Quaterniond blue  = red * Eigen::Quaterniond(0.7071, 0.0, -0.7071, 0.0); 
	  Eigen::Quaterniond green = red * Eigen::Quaterniond(0.7071, 0.0, 0.0, 0.7071);
	  std::cout << "Publishing arrow marker at c: " << c << std::endl;
	  std::cout << "Becase of truncation, c*1000: " << std::trunc(c*1000) << ", fmod: " << fmod(std::trunc(c*1000), 200) << std::endl;
	  axis_marker.pose = current_reference.pose.pose;
	  axis_marker.header.stamp = this->get_clock()->now();

	  arrow_id++;
	  axis_marker.color.r = 1.0;
	  axis_marker.color.g = 0.0;
	  axis_marker.color.b = 0.0;
	  axis_marker.pose.orientation.x = red.x();
	  axis_marker.pose.orientation.y = red.y();
	  axis_marker.pose.orientation.z = red.z();
	  axis_marker.pose.orientation.w = red.w();
	  axis_marker.id = arrow_id;
	  axis_marker.header.stamp = this->get_clock()->now();
	  axis_array_marker.markers.push_back(axis_marker);
	  
	  arrow_id++;
	  axis_marker.color.r = 0.0;
	  axis_marker.color.g = 1.0;
	  axis_marker.color.b = 0.0;
	  axis_marker.pose.orientation.x = green.x();
	  axis_marker.pose.orientation.y = green.y();
	  axis_marker.pose.orientation.z = green.z();
	  axis_marker.pose.orientation.w = green.w();
	  axis_marker.id = arrow_id;
	  axis_marker.header.stamp = this->get_clock()->now();
	  axis_array_marker.markers.push_back(axis_marker);
	  
	  arrow_id++;
	  axis_marker.color.r = 0.0;
	  axis_marker.color.g = 0.0;
	  axis_marker.color.b = 1.0;
	  axis_marker.pose.orientation.x = blue.x();
	  axis_marker.pose.orientation.y = blue.y();
	  axis_marker.pose.orientation.z = blue.z();
	  axis_marker.pose.orientation.w = blue.w();
	  axis_marker.id = arrow_id;
	  axis_marker.header.stamp = this->get_clock()->now();
	  axis_array_marker.markers.push_back(axis_marker);
	  viz_array_publisher->publish(axis_array_marker);
	}
	
	// Publish now for debugging
	current_reference.header.stamp = this->get_clock()->now();
	current_reference.header.frame_id = "world";
	reference_publisher->publish(current_reference);
      }   
      
      c += period / segment_duration;
    }

  private:
    const double period = 0.02; 
    double       c = 0.0;
    double       segment_duration;
    std::chrono::duration<double> freq = std::chrono::duration<double>(period);

    long knots_received = 0;
    long arrow_id = 0;
    
    std::queue<nav_msgs::msg::Odometry> reference_queue;
    nav_msgs::msg::Odometry             current_reference;
    visualization_msgs::msg::Marker     path_marker;
    visualization_msgs::msg::Marker     axis_marker;
    visualization_msgs::msg::MarkerArray axis_array_marker;

    nav_msgs::msg::Odometry             r0;
    nav_msgs::msg::Odometry             r1;

    rclcpp::TimerBase::SharedPtr        control_timer;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr knot_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    pause_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_array_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderInterp>());
  rclcpp::shutdown();
  return 0;
}
