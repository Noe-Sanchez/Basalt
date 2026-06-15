ROBOT_NAME=$1
MAX_ROBOTS=$2
MAX_ROW=$3
SEPARATION=$4
i=0

while [ $i -lt $MAX_ROBOTS ]; do
  x_pos=$(((i % MAX_ROW) * SEPARATION))
  y_pos=$(((i / MAX_ROW) * SEPARATION))

  gz service -s /world/empty/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "type: 2, name: \"$(echo $ROBOT_NAME)_$((i+1))\""
  
  gz service -s /world/empty/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "sdf_filename: \"$(echo $ROBOT_NAME)\", name: \"$(echo $ROBOT_NAME)_$((i+1))\", pose: {position: {x: $x_pos, y: $y_pos, z: 0.5}}" \
  
  i=$((i + 1))
done 
