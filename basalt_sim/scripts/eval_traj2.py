#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import math

class EvalTrajectory2(Node):
  def __init__(self) -> None:
    super().__init__('eval_trajectory2_node')
    
    self.odom_publisher   = self.create_publisher(Odometry, '/control_1/reference/pose', 10)
    #self.odom_publisher   = self.create_publisher(Odometry, '/leader/state', 10)
    self.wrench_publisher = self.create_publisher(Wrench,   '/control_1/feedforward',    10)
    self.odom = Odometry()
    self.wrench = Wrench()

    self.timer = self.create_timer(0.01, self.timer_callback)
    self.time = 0

    # Transform broadcaster
    self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)
    self.ref_tf = TransformStamped()
    self.ref_tf.header.frame_id = 'world'
    self.ref_tf.child_frame_id = 'leader'

  def timer_callback(self) -> None:
    # Publish Odometry
    a = 2.0
    c = 0.25
    self.odom.pose.pose.position.x = a * math.cos(c*self.time)
    self.odom.pose.pose.position.y = a * math.sin(c*self.time) 
    self.odom.pose.pose.position.z = 2.0
    self.odom.pose.pose.orientation.x = 0.0
    self.odom.pose.pose.orientation.y = 0.0
    self.odom.pose.pose.orientation.z = math.sin(c*self.time/2)
    self.odom.pose.pose.orientation.w = math.cos(c*self.time/2)
    self.odom.twist.twist.linear.x = -a*c*math.sin(c*self.time)
    self.odom.twist.twist.linear.y =  a*c*math.cos(c*self.time)
    self.odom.twist.twist.linear.z = 0.0
    self.odom.twist.twist.angular.z = c


    self.wrench.force.x = -a*(c**2)*math.cos(c*self.time)
    self.wrench.force.y = -a*(c**2)*math.sin(c*self.time)
    self.wrench.force.z = 0.0

    self.odom_publisher.publish(self.odom)
    self.wrench_publisher.publish(self.wrench)

    # All 0s for torques and orientations, for now

    self.ref_tf.header.stamp = self.get_clock().now().to_msg()
    self.ref_tf.transform.translation.x = self.odom.pose.pose.position.x
    self.ref_tf.transform.translation.y = self.odom.pose.pose.position.y
    self.ref_tf.transform.translation.z = self.odom.pose.pose.position.z
    self.ref_tf.transform.rotation.x = self.odom.pose.pose.orientation.x
    self.ref_tf.transform.rotation.y = self.odom.pose.pose.orientation.y
    self.ref_tf.transform.rotation.z = self.odom.pose.pose.orientation.z
    self.ref_tf.transform.rotation.w = self.odom.pose.pose.orientation.w
    
    #self.transform_broadcaster.sendTransform(self.ref_tf)


    self.time += 0.01


def main(args=None) -> None:
    print('Publishing evaluation trajectory 2...')
    rclpy.init(args=args)
    eval_traj = EvalTrajectory2()
    rclpy.spin(eval_traj)
    eval_traj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
