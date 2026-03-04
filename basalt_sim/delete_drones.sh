MAX_DRONES=$1
i=0

while [ $i -lt $MAX_DRONES ]; do
  gz service -s /world/empty/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "type: 2, name: \"x500_$((i+1))\""
  
  i=$((i + 1))
done 
