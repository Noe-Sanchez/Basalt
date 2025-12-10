#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench 
from nav_msgs.msg import Odometry
import time
import math

class EvalTrajectory2(Node):
  def __init__(self) -> None:
    super().__init__('eval_trajectory2_node')
    
    #self.odom_publisher   = self.create_publisher(Odometry, '/control_1/reference/pose', 10)
    self.odom_publisher   = self.create_publisher(Odometry, '/leader/state', 10)
    self.wrench_publisher = self.create_publisher(Wrench,   '/control_1/feedforward',    10)
    self.odom = Odometry()
    self.wrench = Wrench()

    self.timer = self.create_timer(0.01, self.timer_callback)
    self.time = 0

  def timer_callback(self) -> None:
    # Publish Odometry
    a = 2.0
    c = 0.25
    self.odom.pose.pose.position.x = a * math.cos(c*self.time)
    self.odom.pose.pose.position.y = a * math.sin(c*self.time) 
    self.odom.pose.pose.position.z = 2.0
    self.odom.twist.twist.linear.x = -a*c*math.sin(c*self.time)
    self.odom.twist.twist.linear.y =  a*c*math.cos(c*self.time)
    self.odom.twist.twist.linear.z = 0.0

    self.wrench.force.x = -a*(c**2)*math.cos(c*self.time)
    self.wrench.force.y = -a*(c**2)*math.sin(c*self.time)
    self.wrench.force.z = 0.0

    self.odom_publisher.publish(self.odom)
    self.wrench_publisher.publish(self.wrench)

    # All 0s for torques and orientations, for now


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
