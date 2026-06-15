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

def quat_mult(q1, q2):
    """
    Multiplies two quaternions [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def slerp_and_derivative(q1, q2, t):
    """
    Computes the SLERP and its first derivative for two quaternions.
    
    Args:
        q1, q2: (4,) numpy arrays representing unit quaternions.
        t: float, interpolation parameter in [0, 1].
        
    Returns:
        q_t: (4,) numpy array, the interpolated quaternion.
        q_dot: (4,) numpy array, the time derivative of the quaternion.
    """
    # Compute cosine of the angle between quaternions
    dot = np.dot(q1, q2)

    # Ensure shortest path by negating q2 if dot product is negative
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    # Clamp to avoid floating point errors with arccos
    dot = np.clip(dot, -1.0, 1.0)
    theta = np.arccos(dot)

    # Fallback to LERP if quaternions are extremely close
    if dot > 0.9995:
        q_t = q1 + t * (q2 - q1)
        q_t /= np.linalg.norm(q_t)
        q_dot = q2 - q1
        return q_t, q_dot

    sin_theta = np.sin(theta)
    
    # Compute SLERP factors
    a = np.sin((1 - t) * theta) / sin_theta
    b = np.sin(t * theta) / sin_theta
    q_t = a * q1 + b * q2

    # Compute derivative factors
    a_dot = -theta * np.cos((1 - t) * theta) / sin_theta
    b_dot = theta * np.cos(t * theta) / sin_theta
    q_dot = a_dot * q1 + b_dot * q2

    return q_t, q_dot

def get_angular_velocity(q, q_dot, frame='global'):
    """
    Converts a unit quaternion and its derivative to a 3D angular velocity vector.
    
    Args:
        q: (4,) numpy array, unit quaternion [w, x, y, z].
        q_dot: (4,) numpy array, time derivative of the quaternion.
        frame: str, 'global' or 'local'.
        
    Returns:
        omega: (3,) numpy array, 3D angular velocity [x, y, z] in rad/s.
    """
    # The conjugate of a unit quaternion [w, x, y, z] is [w, -x, -y, -z]
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    
    if frame == 'global':
        # omega_quat = 2 * q_dot * q^*
        omega_quat = 2.0 * quat_mult(q_dot, q_conj)
    elif frame == 'local':
        # omega_quat = 2 * q^* * q_dot
        omega_quat = 2.0 * quat_mult(q_conj, q_dot)
    else:
        raise ValueError("Frame must be 'global' or 'local'")
        
    # The scalar part (omega_quat[0]) will be practically zero. 
    # The 3D angular velocity is the vector part (indices 1, 2, 3).
    return omega_quat[1:]

class EvalTrajectory2(Node):
  def __init__(self) -> None:
    super().__init__('eval_trajectory3_node')
    
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
    self.odom.pose.pose.position.z = 4.0
    self.odom.twist.twist.linear.x = -a*c*math.sin(c*self.time)
    self.odom.twist.twist.linear.y =  a*c*math.cos(c*self.time)
    self.odom.twist.twist.linear.z = 0.0
    #self.odom.twist.twist.angular.z = c
    
    #self.odom.pose.pose.orientation.x = 0.0
    #self.odom.pose.pose.orientation.y = 0.0
    #self.odom.pose.pose.orientation.z = math.sin(c*self.time/2)
    #self.odom.pose.pose.orientation.w = math.cos(c*self.time/2)

    # Quaternion slerp for orientation
    q1 = np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion
    q2 = np.array([0.354, 0.354, 0.146, 0.85])
    q3 = np.array([0, 0, 0.707, -0.707])
    q4 = np.array([1.0, 0.0, 0.0, 0.0])
    #q1 = nq.quaternion(1.0, 0.0, 0.0, 0.0)  # Identity quaternion
    #q2 = nq.quaternion(0.85, 0.146, 0.354, 0.354)
    # Loop will go on infinitely, so we need modulo to wrap around t between 0 and 1
    #t = (self.time * c/8) % 1.0
    t = (self.time * c/16) % 1.0
    # Dilate time to slow down the rotation
    #t *= c

    # Quaternion slerp using exponential map
    # Do several 
    #q_interp, q_dot = slerp_and_derivative(q1, q2, t)
    #omega = get_angular_velocity(q_interp, q_dot, frame='global')
    if t < 0.25:
      q_interp, q_dot = slerp_and_derivative(q1, q2, 4*t) 
      omega = get_angular_velocity(q_interp, q_dot, frame='global')
    elif t < 0.5: 
      q_interp, q_dot = slerp_and_derivative(q2, q3, 4*(t-0.25))
      omega = get_angular_velocity(q_interp, q_dot, frame='global')
    elif t < 0.75:
      q_interp, q_dot = slerp_and_derivative(q3, q4, 4*(t-0.5))
      omega = get_angular_velocity(q_interp, q_dot, frame='global')
    else:
      q_interp, q_dot = slerp_and_derivative(q4, q1, 4*(t-0.75))
      omega = get_angular_velocity(q_interp, q_dot, frame='global')

    self.odom.pose.pose.orientation.x = q_interp[0]
    self.odom.pose.pose.orientation.y = q_interp[1]
    self.odom.pose.pose.orientation.z = q_interp[2]
    self.odom.pose.pose.orientation.w = q_interp[3]
    #self.odom.twist.twist.angular.x = omega[0]
    #self.odom.twist.twist.angular.y = omega[1]
    #self.odom.twist.twist.angular.z = omega[2]

    self.wrench.force.x = -a*(c**2)*math.cos(c*self.time)
    self.wrench.force.y = -a*(c**2)*math.sin(c*self.time)
    self.wrench.force.z = 0.0

    self.odom.header.stamp = self.get_clock().now().to_msg()
    self.odom.header.frame_id = 'world'

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
    print('Publishing evaluation trajectory 3...')
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
