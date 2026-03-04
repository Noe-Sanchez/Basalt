MAX_DRONES=$1
MAX_ROW=$2
SEPARATION=$3
i=0

while [ $i -lt $MAX_DRONES ]; do
  x_pos=$(((i % MAX_ROW) * SEPARATION))
  y_pos=$(((i / MAX_ROW) * SEPARATION))

  gz service -s /world/empty/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "type: 2, name: \"x500_$((i+1))\""
  
  gz service -s /world/empty/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "sdf_filename: \"x500\", name: \"x500_$((i+1))\", pose: {position: {x: $x_pos, y: $y_pos}}" \
  
  i=$((i + 1))
done 
