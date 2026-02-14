MAX_DRONES=$1
i=0

while [ $i -lt $MAX_DRONES ]; do
  ros2 param set /control_node_$((i+1)) tf_namespace "x500_$((i+1))"
  
  i=$((i + 1))
done 
