ROBOT_NAME=$1
MAX_DRONES=$2
i=0

while [ $i -lt $MAX_DRONES ]; do
  ros2 param set /control_node_$((i+1)) tf_namespace "$(echo $ROBOT_NAME)_$((i+1))"
  
  i=$((i + 1))
done 
