#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import serial
import re
import math

class SerialMarkerPublisher(Node):
    def __init__(self):
        super().__init__('serial_marker_publisher')
        
        # Publisher
        self.publisher = self.create_publisher(Marker, 'marker', 10)
        
        # Serial port
        self.serial_port = serial.Serial('/dev/ttyV0', 115200, timeout=1)
        
        # Timer (100ms = 0.1s)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        try:
            # Read line from serial
            line = self.serial_port.readline().decode('utf-8').strip()
            
            # Parse: X: 0.040, Y: -0.212, Z: -10.034
            match = re.search(r'X:\s*([-+]?\d*\.?\d+),\s*Y:\s*([-+]?\d*\.?\d+),\s*Z:\s*([-+]?\d*\.?\d+)', line)
            
            if match:
                x, y, z = map(float, match.groups())
                
                # Normalize the vector
                magnitude = math.sqrt(x**2 + y**2 + z**2)
                if magnitude > 0:
                    x_norm = x / magnitude
                    y_norm = y / magnitude
                    z_norm = z / magnitude
                else:
                    x_norm = y_norm = z_norm = 0.0
                
                # Create Marker message
                msg = Marker()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'
                msg.ns = 'serial_arrow'
                msg.id = 0
                msg.type = Marker.ARROW
                msg.action = Marker.ADD
                
                # Arrow from origin to normalized point
                start = Point()
                start.x = 0.0
                start.y = 0.0
                start.z = 0.0
                
                end = Point()
                end.x = x_norm
                end.y = -y_norm
                end.z = -z_norm
                
                msg.points = [start, end]
                
                # Arrow appearance
                msg.scale.x = 0.05  # Shaft diameter
                msg.scale.y = 0.1   # Head diameter
                msg.scale.z = 0.0   # Not used for ARROW with points
                
                msg.color.r = 1.0
                msg.color.g = 0.0
                msg.color.b = 0.0
                msg.color.a = 1.0
                
                self.publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
