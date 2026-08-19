import argparse
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  num_drones = int(sys.argv[4:][0].split('=')[1])
  print(f"Launching simulation with {num_drones} drones.")
  
  nodes = []

  for i in range(num_drones):
    control_node = Node(
      package="basalt_sim",
      executable="control_node",
      name=f"control_node_{i+1}",
      output="screen",
      remappings=[("/control_1/reference/pose",   f"/control_{i+1}/reference/pose"),
                  ("/model/x500_1/odometry",      f"/model/x500_{i+1}/odometry"),
                  ("/control_1/control_force",    f"/control_{i+1}/control_force"),
                  ("/control_1/error",            f"/control_{i+1}/error"),
                  ("/x500_1/command/motor_speed", f"/x500_{i+1}/command/motor_speed"),],
    )
    nodes.append(control_node)

  mujoco_sim_node = Node(package="basalt_sim",
                         executable="mujoco_node",
                         name="mujoco_node",
                         output="screen",
                         parameters=[{"num_drones": num_drones}],
                        )
  nodes.append(mujoco_sim_node)

  return LaunchDescription(nodes)

if __name__ == "__main__":
  generate_launch_description()

