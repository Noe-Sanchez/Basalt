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

  bridge_config_path = os.path.join(get_package_share_directory("basalt_sim"), "config", "omni_config.yaml")
  with open(bridge_config_path, "w") as f:
    f.write("")

  for i in range(num_drones):
    control_node = Node(
      package="basalt_sim",
      executable="full_control_node",
      name=f"control_node_{i+1}",
      output="screen",
      remappings=[("/control_1/reference/pose",   f"/control_{i+1}/reference/pose"),
                  ("/model/omni_1/odometry",      f"/model/omni_{i+1}/odometry"),
                  ("/control_1/control_force",    f"/control_{i+1}/control_force"),
                  ("/control_1/error",            f"/control_{i+1}/error"),
                  ("/omni_1/command/motor_speed", f"/omni_{i+1}/command/motor_speed"),],
    )
    nodes.append(control_node)

    # Append entries to ros_gz_bridge config file
    bridge_config_path = os.path.join(get_package_share_directory("basalt_sim"), "config", "omni_config.yaml")
    with open(bridge_config_path, "a") as f:
      f.write(f'- topic_name:     "/omni_{i+1}/command/motor_speed"\n  ros_type_name:  "actuator_msgs/msg/Actuators"\n  gz_type_name:   "gz.msgs.Actuators"\n  direction:      ROS_TO_GZ\n\n- topic_name:     "/model/omni_{i+1}/odometry"\n  ros_type_name:  "nav_msgs/msg/Odometry"\n  gz_type_name:   "gz.msgs.Odometry"\n  direction:      GZ_TO_ROS\n\n')

  ros_gz_bridge_node = Node(package="ros_gz_bridge",
                            executable="parameter_bridge",
                            name="ros_gz_bridge",
                            output="screen",
                            parameters=[{"config_file": os.path.join(get_package_share_directory("basalt_sim"), "config", "omni_config.yaml")}],
                           )

  nodes.append(ros_gz_bridge_node)

  return LaunchDescription(nodes)

if __name__ == "__main__":
  generate_launch_description()

