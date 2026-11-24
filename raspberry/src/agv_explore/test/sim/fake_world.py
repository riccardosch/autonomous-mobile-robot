#!/usr/bin/env python3
"""Fake world for the explorer: a 10 x 8 m house, no robot needed.

Stands in for everything the explorer talks to:
  /map                      slam_toolbox: the cells seen so far (ray casting, 2.5 m, stops at walls)
  /global_costmap/costmap   Nav2: walls 100, robot radius 99, cost decaying to 0 at 0.45 m
  TF map -> base_link       where the fake robot is
  navigate_to_pose          Nav2: after 1 s the robot jumps to the goal (aborts in a wall)
  slam_toolbox/serialize_map  answers "ok"
Run it on its own ROS domain (see run_sim.sh) so it never meets the real robot.
FAIL_FIRST=n makes the first n goals fail, to test the explorer's retries.
"""
import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import TransformBroadcaster

R = 0.05                     # m per cell
W, H = 200, 160              # 10 x 8 m
TRUE = np.zeros((H, W), np.int8)
TRUE[0, :] = TRUE[-1, :] = TRUE[:, 0] = TRUE[:, -1] = 100   # outer walls
TRUE[:, 100] = 100
TRUE[70:90, 100] = 0                                        # middle wall, 1 m door
TRUE[80, 100:] = 100
TRUE[80, 150:170] = 0                                       # right room split, 1 m door
FAIL_FIRST = int(os.environ.get('FAIL_FIRST', '0'))


class World(Node):
    def __init__(self):
        super().__init__('fake_world')
        self.x, self.y = 2.0, 4.0
        self.known = np.zeros((H, W), bool)
        self.goals = self.preempted = 0
        self.current = None
        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.cost_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', latched)
        self.tf = TransformBroadcaster(self)
        self.reveal()
        self.create_timer(0.5, self.publish, callback_group=cb)
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.create_service(SerializePoseGraph, '/slam_toolbox/serialize_map', self.serialize,
                            callback_group=cb)

    def reveal(self):
        cx, cy = int(self.x / R), int(self.y / R)
        for a in np.linspace(0, 2 * math.pi, 720, endpoint=False):
            for d in range(int(2.5 / R)):
                c, r = int(cx + d * math.cos(a)), int(cy + d * math.sin(a))
                if not (0 <= c < W and 0 <= r < H):
                    break
                self.known[r, c] = True
                if TRUE[r, c] == 100:
                    break

    def publish(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = R
        m.info.width, m.info.height = W, H
        m.data = np.where(self.known, TRUE, -1).astype(np.int8).flatten().tolist()
        self.map_pub.publish(m)
        d = distance_transform_edt(~(self.known & (TRUE == 100))) * R
        cost = np.where(d <= 0.0, 100, np.where(d <= 0.17, 99,
                        np.where(d < 0.45, (98 * np.exp(-5 * (d - 0.17))).astype(int), 0)))
        c = OccupancyGrid()
        c.header, c.info = m.header, m.info
        c.data = np.where(self.known, cost, -1).astype(np.int8).flatten().tolist()
        self.cost_pub.publish(c)
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

    def execute(self, gh):
        p = gh.request.pose.pose.position
        self.goals += 1
        self.current = gh
        for _ in range(10):
            if gh.is_cancel_requested:
                gh.canceled()
                return NavigateToPose.Result()
            if self.current is not gh:   # replaced by a newer goal
                self.preempted += 1
                gh.abort()
                return NavigateToPose.Result()
            time.sleep(0.1)
        if self.goals <= FAIL_FIRST or TRUE[int(p.y / R), int(p.x / R)] == 100:
            gh.abort()
            return NavigateToPose.Result()
        self.x, self.y = p.x, p.y
        self.reveal()
        gh.succeed()
        return NavigateToPose.Result()

    def serialize(self, req, res):
        print('SERIALIZE', req.filename, flush=True)
        res.result = 0
        return res


def main():
    rclpy.init()
    w = World()
    ex = MultiThreadedExecutor()
    ex.add_node(w)
    end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 200)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))   # run_sim.sh ends us with kill
    while time.time() < end and not stop:
        ex.spin_once(timeout_sec=0.1)
    free = TRUE == 0
    print('free space seen: %.1f%%, goals %d, replaced %d'
          % (100 * (w.known & free).sum() / free.sum(), w.goals, w.preempted), flush=True)


if __name__ == '__main__':
    main()
