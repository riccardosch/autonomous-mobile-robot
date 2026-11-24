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

[frontiers.cpp](src/frontiers.cpp) (no ROS, tested in [test/](test/test_frontiers.cpp)) and [explorer_node.cpp](src/explorer_node.cpp):

1. Every second, on the `/map` from `slam_toolbox`, find the **frontier cells**: free cells with an unknown neighbour. Group the touching ones into frontiers; drop those shorter than 40 cm.
2. Pick one: `score = 0.3 × length [m] − distance [m] − 0.5 × turn [rad]`, where *turn* is the angle between the robot heading and the frontier. Close frontiers ahead of the robot win, so it sweeps the house instead of jumping around.
3. The goal is not the frontier cell itself (often right against a wall): the robot stops **0.8 m in front of the frontier, facing it** (the frontier's normal points into the unknown), so the Kinect (0.5-1.5 m) looks straight into a gap before the robot drives in. The spot must be free on the map and comfortable on the global costmap (cost ≤ 60/100); if there is none there, any such spot within 0.8 m of the frontier; if none, the frontier is blacklisted.
4. Send it to Nav2 (`navigate_to_pose`). If the frontier is seen from afar on the way, the next goal is sent at once and **replaces** the running one inside Nav2, without stopping.
5. On arrival it goes on at once to the next frontier; 3 s later (the map is updated every 2 s) the one it arrived at is checked. Still open after looking at it from the standoff (something hides it): next time the robot **drives right up to it**. Still open even up close: the LiDAR cannot see into it (glass, a gap under furniture), so it is **blacklisted**.
6. A frontier that Nav2 cannot reach (or with no room for the robot, or taking more than 2 minutes) is **retried**: it rests for 20 s while the robot goes elsewhere (meanwhile old costmap marks fade and the map improves), up to 3 attempts, then it is blacklisted. Blacklisted points ban frontiers within 0.5 m. While a frontier waits for another attempt the exploration is not declared finished.
7. No frontier left for 5 s in a row → save the map in `~/maps/<name>`: `.pgm` + `.yaml` written by the explorer from its own copy of `/map` (same format as Nav2's `map_saver`; `slam_toolbox/save_map` failed once when `/map` took more than 2 s to reach `map_saver`), and `.posegraph` + `.data` via `slam_toolbox/serialize_map` (to continue the map or localize on it later).

Parameters: [config/explorer.yaml](config/explorer.yaml). What the explorer decides is in its log lines (`explorer` in the launch terminal, also in `~/.ros/log/explorer_*.log`).

## Simulation

[test/sim/](test/sim/): a fake 10 × 8 m house that stands in for SLAM, Nav2 and TF, on ROS domain 77 so it never meets the real robot.

```bash
cd ~/agv/autonomous-mobile-robot/raspberry && source install/setup.bash
src/agv_explore/test/sim/run_sim.sh                 # explorer decisions + share of the house seen
FAIL_FIRST=2 src/agv_explore/test/sim/run_sim.sh    # first two goals fail: retries
```

It checks the explorer's logic (frontiers, choice, blacklist, retries, end, map file), not driving: the fake robot jumps to each goal.

## Limits

- The map comes from the LiDAR only (scan plane ~13 cm). Low obstacles seen by the Kinect are avoided by Nav2 but are not part of the saved map.
- Closed doors end the exploration there: open the doors you want mapped.
- Gaps narrower than the robot (34 cm) plus a few cm cannot be passed.

## Status

- 2026-11-21: unit tests pass. Checked on the Pi against a simulated house (10 × 8 m, two rooms and two doors; a fake Nav2 moves the robot to each goal and the map is revealed by ray casting within 2.5 m): 20 goals, 100% of the free space seen in 1.8 min, map saved. First run on the robot (2026-11-21, `prova1`): it explored and saved the map in 4 min, but slowly, with stops between goals, 4 of 10 goals that Nav2 could not plan to (goal on the frontier cell, against a wall) and passages refused because of Kinect marks that never faded. Changed the same day: safe goal spot, goal replacement without stopping, heading-aware choice, Kinect layer that forgets, smaller margins, 0.2 m/s. Simulation after the changes: 20 goals, 100% seen in 0.8 min (was 1.8). Second robot run (`prova2`): the robot stopped with `collision ahead` 5-7 cm from chair legs and could not recover (spin failed). Then (block 3c): frontal approach, recovery that backs up and looks again, two radii (planner optimistic, local costmap real), Kinect marks kept until seen empty. Simulation: 17 goals, none blacklisted, 100% in 0.7 min. `prova3`: room explored in 2 min. Next run: stopped at an open door — Nav2 could not plan to it once (the Kinect was still on the old parameters, see agv_perception_depth) and the explorer blacklisted it at once, then `map_saver` failed. Now: retries, own map writer.
