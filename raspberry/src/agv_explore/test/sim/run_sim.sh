#!/bin/bash
# Explorer against the fake house, on ROS domain 77 (never meets the real robot).
#   test/sim/run_sim.sh [out_dir]          FAIL_FIRST=2 test/sim/run_sim.sh   (retries)
# Prints the explorer's decisions and the share of the house it saw.
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-/tmp/agv_explore_sim}
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash
source "$HERE/../../../../install/setup.bash"
export ROS_DOMAIN_ID=77
python3 "$HERE/fake_world.py" 200 > "$OUT/world.out" 2>&1 &
WORLD=$!
sleep 2
timeout 190 ros2 run agv_explore explorer --ros-args \
  --params-file "$(ros2 pkg prefix agv_explore)/share/agv_explore/config/explorer.yaml" \
  -p map_dir:="$OUT/maps" -p map_name:=sim > "$OUT/explorer.out" 2>&1
kill $WORLD 2>/dev/null; wait $WORLD 2>/dev/null
sed 's/^\[[A-Z]*\] \[[0-9.]*\] \[explorer\]: //' "$OUT/explorer.out"
cat "$OUT/world.out"
