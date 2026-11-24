# agv_explore

Autonomous exploration (mission mode 1): the robot drives around the house by itself, with no goals from the user, until the map is complete, then saves it.

```bash
source ~/kinect/install/setup.bash && source ~/microros_ws/install/setup.bash
source ~/agv/autonomous-mobile-robot/raspberry/install/setup.bash
ros2 launch agv_explore explore.launch.py map_name:=casa
```

It starts [agv_bringup/nav.launch.py](../agv_bringup/launch/nav.launch.py) (robot, SLAM, Kinect depth, Nav2) and the `explorer` node. Keep the robot still while it starts. Ctrl+C stops everything (the wheel ESP32 stops the wheels 0.5 s after the last command).

| Argument | Default | |
|---|---|---|
| `map_name` | empty = `house_<date>_<time>` | Name of the saved map in `~/maps` |
| `time_limit_min` | `20.0` | Stops and saves the map after this time anyway (`5` and `5.0` both work) |
| `kinect` | `true` | Kinect depth in the costmaps (low obstacles) |

## How it works

Once a second, on the `/map` from `slam_toolbox` and the Nav2 global costmap:

1. **Frontiers**: free cells with an unknown neighbour, grouped when they touch; those shorter than 40 cm are dropped. Each has a direction pointing into the unknown.
2. **Choice**: `score = 0.3 × length [m] − distance [m] − 0.5 × turn [rad]` (*turn*: angle between the robot heading and the frontier). Close frontiers ahead of the robot win, so it sweeps the house instead of jumping around.
3. **Goal spot**: the robot stops **0.8 m in front of the frontier, facing it**, so the Kinect (0.5-1.5 m) looks into a gap before the robot drives in. The spot must be free on the map and at least ~4 cm beyond the robot's radius on the global costmap (cost ≤ 85/100). If there is no such spot in front, or if the frontier must be approached up close (step 5), a spot near the frontier itself.
4. **Driving**: the goal goes to Nav2 (`navigate_to_pose`). If the frontier is seen from afar on the way, the next goal replaces the running one inside Nav2, without stopping.
5. **Arrival**: the robot goes on at once; 3 s later (the map updates every 2 s) the frontier is judged. Explored: forgotten. Still open after looking at it from 0.8 m (something hides it): next time the robot **drives up to it**. Still open even up close (glass, a gap under furniture): **blacklisted**.
6. **Failures** (Nav2 aborts, no room near the frontier, more than 2 minutes): the frontier rests for 20 s while the robot goes elsewhere (old costmap marks fade, the map improves), up to 3 attempts, then it is blacklisted. Two frontier points within 0.5 m count as the same frontier.
7. **End**: no frontier left for 5 s in a row, and none waiting for another attempt → the map is saved in `~/maps/<name>`: `.pgm` + `.yaml` (same format as Nav2's `map_saver`, written by the explorer from its copy of `/map`) and `.posegraph` + `.data` from `slam_toolbox/serialize_map` (to continue the map or localize on it later). Also after `time_limit_min`.

The explorer's decisions are in its log lines (launch terminal, and `~/.ros/log/explorer_*.log`). Parameters: [config/explorer.yaml](config/explorer.yaml).

## Code

The node only talks to ROS; the decisions are in ROS-free modules with unit tests ([test/](test/)):

| File | Does |
|---|---|
| [grid.hpp](include/agv_explore/grid.hpp) | `Grid` (an `OccupancyGrid` without ROS), `Point`, `Robot` |
| [frontiers.hpp](include/agv_explore/frontiers.hpp) | find frontiers; is a frontier still open |
| [goal_selector.hpp](include/agv_explore/goal_selector.hpp) | score, order, goal spot in front of or near the frontier |
| [frontier_memory.hpp](include/agv_explore/frontier_memory.hpp) | failures and retries, blacklist, frontiers to approach up close, arrivals waiting to be judged |
| [map_file.hpp](include/agv_explore/map_file.hpp) | `.pgm` + `.yaml` map files |
| [explorer_node.cpp](src/explorer_node.cpp) | parameters, topics, Nav2 action, TF, the state machine (`WaitingForData → Idle ⇄ Driving → Finished`, described at the top of the file) |

```bash
colcon build --packages-select agv_explore
colcon test --packages-select agv_explore && colcon test-result --verbose
```

## Simulation

[test/sim/](test/sim/): a fake 10 × 8 m house that stands in for SLAM, Nav2 and TF, on ROS domain 77 so it never meets the real robot.

```bash
cd ~/agv/autonomous-mobile-robot/raspberry && source install/setup.bash
src/agv_explore/test/sim/run_sim.sh                 # explorer decisions + share of the house seen
FAIL_FIRST=2 src/agv_explore/test/sim/run_sim.sh    # first two goals fail: retries
```

It checks the explorer's logic (frontiers, choice, memory, end, map file), not driving: the fake robot jumps to each goal. Expected: 17 goals, 100% of the house seen, nothing blacklisted (19 goals with `FAIL_FIRST=2`).

## Limits

- The map comes from the LiDAR only (scan plane ~13 cm). Low obstacles seen by the Kinect are avoided by Nav2 but are not part of the saved map.
- Closed doors end the exploration there: open the doors you want mapped.
- If the robot hits something and the wheels slip, the SLAM can lose its place and the map becomes inconsistent; there is no relocalization yet.

## Status

Unit tests and the simulation pass. On the robot it has explored and saved single rooms by itself. History of the robot runs and of the changes in [docs/devlog.md](../../../docs/devlog.md).
