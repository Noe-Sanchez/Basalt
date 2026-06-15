ROBOT_NAME=$1
MAX_ROBOTS=$2
i=0

while [ $i -lt $MAX_ROBOTS ]; do
  gz service -s /world/empty/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "type: 2, name: \"$(echo $ROBOT_NAME)_$((i+1))\""

  i=$((i + 1))
done 
