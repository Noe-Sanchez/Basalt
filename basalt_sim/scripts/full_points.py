#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import math
import numpy as np

class FullPoints(Node):
  def __init__(self) -> None:
    super().__init__('full_points_node')
    
    self.odom_publisher   = self.create_publisher(Odometry, '/reference/new_knot', 10)

    knot = Odometry()

    knot.header.frame_id = 'world'

    knot.pose.pose.position.x = 1.0
    knot.pose.pose.position.y = 0.0
    knot.pose.pose.position.z = 3.0
    knot.pose.pose.orientation.x = 0.0
    knot.pose.pose.orientation.y = 0.0
    knot.pose.pose.orientation.z = 0.0
    knot.pose.pose.orientation.w = 1.0
    knot.twist.twist.linear.x = 0.0
    knot.twist.twist.linear.y = 0.5
    knot.twist.twist.linear.z = 0.0
    knot.twist.twist.angular.x = 0.0
    knot.twist.twist.angular.y = 0.0
    knot.twist.twist.angular.z = 0.0
    knot.header.stamp = self.get_clock().now().to_msg()
    self.odom_publisher.publish(knot)
    time.sleep(0.1)

    knot.pose.pose.position.x = 0.0
    knot.pose.pose.position.y = 1.0
    knot.pose.pose.position.z = 4.0
    knot.pose.pose.orientation.x = -0.7071
    knot.pose.pose.orientation.y = 0.0
    knot.pose.pose.orientation.z = 0.0
    knot.pose.pose.orientation.w = 0.7071;
    knot.twist.twist.linear.x = -0.5
    knot.twist.twist.linear.y = 0.0
    knot.twist.twist.linear.z = 0.0
    knot.twist.twist.angular.x = 0.0
    knot.twist.twist.angular.y = 0.0
    knot.twist.twist.angular.z = 0.0
    knot.header.stamp = self.get_clock().now().to_msg()
    self.odom_publisher.publish(knot)
    time.sleep(0.1)
    
    knot.pose.pose.position.x = -1.0
    knot.pose.pose.position.y = 0.0
    knot.pose.pose.position.z = 5.0
    knot.pose.pose.orientation.x =  0.5
    knot.pose.pose.orientation.y = -0.5
    knot.pose.pose.orientation.z = -0.5
    knot.pose.pose.orientation.w = -0.5
    knot.twist.twist.linear.x = 0.0
    knot.twist.twist.linear.y = -0.5
    knot.twist.twist.linear.z = 0.0
    knot.twist.twist.angular.x = 0.0
    knot.twist.twist.angular.y = 0.0
    knot.twist.twist.angular.z = 0.0
    knot.header.stamp = self.get_clock().now().to_msg()
    self.odom_publisher.publish(knot)
    time.sleep(0.1)
    
    knot.pose.pose.position.x = 0.0
    knot.pose.pose.position.y = -1.0
    knot.pose.pose.position.z = 4.0
    knot.pose.pose.orientation.x = 0.0
    knot.pose.pose.orientation.y = 0.0
    knot.pose.pose.orientation.z = 0.7071
    knot.pose.pose.orientation.w = 0.7071
    knot.twist.twist.linear.x = 0.5
    knot.twist.twist.linear.y = 0.0
    knot.twist.twist.linear.z = 0.0
    knot.twist.twist.angular.x = 0.0
    knot.twist.twist.angular.y = 0.0
    knot.twist.twist.angular.z = 0.0
    knot.header.stamp = self.get_clock().now().to_msg()
    self.odom_publisher.publish(knot)
    time.sleep(0.1)
    
    knot.pose.pose.position.x = 1.0
    knot.pose.pose.position.y = 0.0
    knot.pose.pose.position.z = 3.0
    knot.pose.pose.orientation.x = 0.0
    knot.pose.pose.orientation.y = 0.0
    knot.pose.pose.orientation.z = 0.0
    knot.pose.pose.orientation.w = 1.0
    knot.twist.twist.linear.x = 0.0
    knot.twist.twist.linear.y = 0.5
    knot.twist.twist.linear.z = 0.0
    knot.twist.twist.angular.x = 0.0
    knot.twist.twist.angular.y = 0.0
    knot.twist.twist.angular.z = 0.0
    knot.header.stamp = self.get_clock().now().to_msg()
    self.odom_publisher.publish(knot)
    time.sleep(0.1)

    # End node gracefully here
    print('Finished publishing points. Shutting down node...')
    self.destroy_node()

def main(args=None) -> None:
    print('Publishing points...') 
    rclpy.init(args=args)
    full_points = FullPoints()
    rclpy.spin(full_points)
    eval_traj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
