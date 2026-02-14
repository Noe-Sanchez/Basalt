#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench 
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped
import time
import math

class EvalTrajectory(Node):
  def __init__(self) -> None:
    super().__init__('eval_trajectory_node')
    
    #self.odom_publisher   = self.create_publisher(Odometry, '/control_1/reference/pose', 10)
    self.odom_publisher   = self.create_publisher(Odometry,  '/leader/state', 10)
    self.wrench_publisher = self.create_publisher(Wrench,    '/control_1/feedforward',    10)
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
    c = 0.2
    coeff = a/(1+math.sin(c*self.time)*math.sin(c*self.time))
    self.odom.pose.pose.position.x = coeff * math.cos(c*self.time)
    self.odom.pose.pose.position.y = coeff * math.sin(c*self.time)*math.cos(c*self.time)
    self.odom.pose.pose.position.z = 2.0
    self.odom.twist.twist.linear.x = (a*c*math.sin(c*self.time)*(math.sin(c*self.time)*math.sin(c*self.time) - 3) )/( (math.sin(c*self.time)*math.sin(c*self.time) + 1)**2 )
    self.odom.twist.twist.linear.y = (2*a*c*(3*math.cos(2*c*self.time) - 1) )/( (math.cos(2*c*self.time) - 3)**2 )
    self.odom.twist.twist.linear.z = 0.0

    self.wrench.force.x = -(a*c*c*math.cos(c*self.time)*((math.cos(c*self.time)**4) + 10*(math.cos(c*self.time)**2)-8))/( ((math.sin(c*self.time)**2)+1)**3) 
    self.wrench.force.y = ((4*a*c*c)*(3*math.cos(2*c*self.time)+7)*math.sin(2*c*self.time))/((math.cos(2*c*self.time)-3)**3)
    self.wrench.force.z = 0.0

    self.odom_publisher.publish(self.odom)
    self.wrench_publisher.publish(self.wrench)

    # All 0s for torques and orientations, for now
    self.odom.header.stamp = self.get_clock().now().to_msg()
    self.odom.header.frame_id = 'world'

    # Publish Transform
    self.ref_tf.header.stamp = self.get_clock().now().to_msg()
    self.ref_tf.transform.translation.x = self.odom.pose.pose.position.x
    self.ref_tf.transform.translation.y = self.odom.pose.pose.position.y
    self.ref_tf.transform.translation.z = self.odom.pose.pose.position.z
    
    self.transform_broadcaster.sendTransform(self.ref_tf)

    self.time += 0.01


def main(args=None) -> None:
    print('Publishing evaluation trajectory...')
    rclpy.init(args=args)
    eval_traj = EvalTrajectory()
    rclpy.spin(eval_traj)
    eval_traj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
