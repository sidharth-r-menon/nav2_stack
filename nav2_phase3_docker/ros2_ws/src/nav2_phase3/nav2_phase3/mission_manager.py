#!/usr/bin/env python3
"""A small, inspectable frontier exploration state machine that delegates motion to Nav2."""
from collections import deque
import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def yaw_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion(); q.z = math.sin(yaw / 2); q.w = math.cos(yaw / 2); return q


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('min_frontier_cells', 12)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('approach_distance', 0.70)
        self.declare_parameter('finish_mapping_before_approach', False)
        self.min_cells = self.get_parameter('min_frontier_cells').value
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value
        self.standoff = self.get_parameter('approach_distance').value
        self.finish_before_approach = self.get_parameter('finish_mapping_before_approach').value
        self.map = None; self.ball = None; self.state = 'WAITING_FOR_MAP'; self.goal_handle = None
        self.goal_started = None; self.goal_pending = False; self.active_xy = None; self.blacklist = []; self.idle_cycles = 0
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/phase3/ball_pose', self.ball_cb, 10)
        self.status_pub = self.create_publisher(String, '/phase3/mission_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.create_timer(1.0, self.tick)

    def say(self, text):
        if text != self.state:
            self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def map_cb(self, msg): self.map = msg

    def ball_cb(self, msg):
        self.ball = msg
        if not self.finish_before_approach and self.state not in ('BALL_FOUND', 'APPROACHING', 'SUCCEEDED'):
            self.state = 'BALL_FOUND'; self.say('BALL_FOUND: cancelling frontier goal and preparing a stand-off approach')
            if self.goal_handle is not None: self.goal_handle.cancel_goal_async()

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.15))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def tick(self):
        if self.map is None:
            self.state = 'WAITING_FOR_MAP'; self.say('WAITING_FOR_MAP: SLAM Toolbox has not published /map yet'); return
        if not self.nav.server_is_ready():
            self.state = 'WAITING_FOR_NAV2'; self.say('WAITING_FOR_NAV2: waiting for navigate_to_pose action'); return
        if self.goal_handle is not None:
            if self.goal_started and (self.get_clock().now() - self.goal_started).nanoseconds * 1e-9 > self.goal_timeout:
                self.get_logger().warn('Goal timeout; cancelling and blacklisting its area.')
                self.goal_handle.cancel_goal_async(); self.blacklist.append(self.active_xy); self.goal_handle = None
            return
        if self.goal_pending:
            return
        if self.state == 'BALL_FOUND' and self.ball is not None:
            goal = self.approach_goal()
            if goal is not None: self.send_goal(goal, 'APPROACHING'); return
        if self.state in ('SUCCEEDED', 'EXPLORATION_COMPLETE'):
            return
        frontier = self.best_frontier()
        if frontier is None:
            self.idle_cycles += 1
            self.state = 'EXPLORING'; self.say('EXPLORING: no safe frontier in the current map; waiting for map update')
            if self.idle_cycles >= 8:
                self.state = 'EXPLORATION_COMPLETE'; self.say('EXPLORATION_COMPLETE: no reachable frontiers remain')
            return
        self.idle_cycles = 0; self.send_goal(frontier, 'EXPLORING')

    def grid_xy(self, row, col):
        info = self.map.info
        return (info.origin.position.x + (col + .5) * info.resolution,
                info.origin.position.y + (row + .5) * info.resolution)

    def map_cell_free(self, x, y, clearance=2):
        if self.map is None: return False
        info = self.map.info; col = int((x - info.origin.position.x) / info.resolution); row = int((y - info.origin.position.y) / info.resolution)
        data = np.asarray(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        if row < clearance or col < clearance or row >= info.height-clearance or col >= info.width-clearance or data[row, col] > 25 or data[row, col] < 0: return False
        return not np.any(data[row-clearance:row+clearance+1, col-clearance:col+clearance+1] >= 65)

    def best_frontier(self):
        pose = self.robot_xy()
        if pose is None: return None
        info = self.map.info; data = np.asarray(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        free = (data >= 0) & (data <= 25); unknown = data < 0
        adjacent_unknown = np.zeros_like(unknown)
        adjacent_unknown[1:] |= unknown[:-1]; adjacent_unknown[:-1] |= unknown[1:]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]; adjacent_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & adjacent_unknown; seen = np.zeros_like(frontier, dtype=bool); candidates = []
        for row, col in np.argwhere(frontier):
            if seen[row, col]: continue
            queue = deque([(int(row), int(col))]); seen[row, col] = True; group = []
            while queue:
                r, c = queue.popleft(); group.append((r, c))
                for rr, cc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1),(r-1,c-1),(r-1,c+1),(r+1,c-1),(r+1,c+1)):
                    if 0 <= rr < info.height and 0 <= cc < info.width and frontier[rr,cc] and not seen[rr,cc]:
                        seen[rr,cc] = True; queue.append((rr,cc))
            if len(group) < self.min_cells: continue
            center = np.mean(group, axis=0); group.sort(key=lambda p: (p[0]-center[0])**2 + (p[1]-center[1])**2)
            for r, c in group:
                x, y = self.grid_xy(r, c)
                if not self.map_cell_free(x, y, clearance=3): continue
                if any(math.hypot(x-bx, y-by) < .8 for bx, by in self.blacklist): continue
                distance = math.hypot(x-pose[0], y-pose[1])
                if distance > .45: candidates.append((len(group) * info.resolution - .20 * distance, x, y, len(group)))
                break
        if not candidates: return None
        _, x, y, size = max(candidates, key=lambda c: c[0])
        self.publish_frontier_marker(x, y, size)
        goal = PoseStamped(); goal.header.frame_id = 'map'; goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.orientation = yaw_quaternion(math.atan2(y-pose[1], x-pose[0])); return goal

    def approach_goal(self):
        robot = self.robot_xy()
        if robot is None: return None
        bx, by = self.ball.pose.position.x, self.ball.pose.position.y
        bearing = math.atan2(by-robot[1], bx-robot[0])
        options = []
        for radius in (self.standoff, self.standoff+.20, self.standoff+.40):
            for offset in (0, .52, -.52, 1.05, -1.05, 1.57, -1.57):
                a = bearing + offset; x, y = bx - radius * math.cos(a), by - radius * math.sin(a)
                if self.map_cell_free(x, y, clearance=4): options.append((math.hypot(x-robot[0], y-robot[1]), x, y))
        if not options:
            self.get_logger().warn('Ball is known, but no mapped free stand-off point is available yet. Continuing exploration.')
            self.state = 'EXPLORING'; return None
        _, x, y = min(options); goal = PoseStamped(); goal.header.frame_id='map'; goal.header.stamp=self.get_clock().now().to_msg()
        goal.pose.position.x=x; goal.pose.position.y=y; goal.pose.orientation=yaw_quaternion(math.atan2(by-y, bx-x)); return goal

    def send_goal(self, pose, state):
        self.state = state; self.say(f'{state}: Nav2 goal x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}')
        goal = NavigateToPose.Goal(); goal.pose = pose
        self.goal_pending = True; self.active_xy = (pose.pose.position.x, pose.pose.position.y)
        future = self.nav.send_goal_async(goal); future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        self.goal_pending = False
        handle = future.result()
        if not handle.accepted:
            self.goal_handle = None; self.blacklist.append(self.active_xy); self.get_logger().warn('Nav2 rejected the goal'); return
        self.goal_handle = handle; self.goal_started = self.get_clock().now(); handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result(); was_approach = self.state == 'APPROACHING'; self.goal_handle = None; self.goal_started = None
        if result.status != GoalStatus.STATUS_SUCCEEDED and self.active_xy is not None: self.blacklist.append(self.active_xy)
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if was_approach:
                self.state = 'SUCCEEDED'; self.say('SUCCEEDED: reached the mapped stand-off point for the red ball')
            else:
                self.state = 'EXPLORING'; self.say('EXPLORING: frontier reached; selecting the next boundary')
        else:
            self.state = 'EXPLORING' if not was_approach else 'BALL_FOUND'; self.say('Goal ended without success; selecting another safe candidate')

    def publish_frontier_marker(self, x, y, size):
        marker = Marker(); marker.header.frame_id='map'; marker.header.stamp=self.get_clock().now().to_msg(); marker.ns='phase3_frontier'; marker.id=1
        marker.type=Marker.SPHERE; marker.action=Marker.ADD; marker.pose.position.x=x; marker.pose.position.y=y; marker.pose.orientation.w=1.0
        marker.scale.x=marker.scale.y=marker.scale.z=.25; marker.color.g=1.0; marker.color.r=.1; marker.color.a=.9
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init(); node = MissionManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()
