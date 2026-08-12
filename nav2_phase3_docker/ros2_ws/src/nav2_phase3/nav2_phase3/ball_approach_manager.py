#!/usr/bin/env python3
"""Keep exploration upstream; only hand off a confirmed perception target to Nav2."""
import math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def yaw_q(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion(); q.z = math.sin(yaw / 2.0); q.w = math.cos(yaw / 2.0); return q


class BallApproachManager(Node):
    def __init__(self):
        super().__init__('ball_approach_manager')
        self.declare_parameter('approach_distance', .70)
        self.standoff = self.get_parameter('approach_distance').value
        self.map = None; self.target = None; self.goal_sent = False
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/phase3/ball_pose', self.ball_cb, 10)
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.status_pub = self.create_publisher(String, '/phase3/mission_status', 10)

    def map_cb(self, msg): self.map = msg

    def say(self, message):
        self.get_logger().info(message); self.status_pub.publish(String(data=message))

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception: return None

    def free(self, x, y, clearance=4):
        if self.map is None: return False
        i = self.map.info; c = int((x-i.origin.position.x)/i.resolution); r = int((y-i.origin.position.y)/i.resolution)
        data = np.asarray(self.map.data, dtype=np.int16).reshape(i.height, i.width)
        if r < clearance or c < clearance or r >= i.height-clearance or c >= i.width-clearance: return False
        return data[r, c] >= 0 and data[r, c] <= 25 and not np.any(data[r-clearance:r+clearance+1, c-clearance:c+clearance+1] >= 65)

    def ball_cb(self, pose):
        if self.goal_sent: return
        self.target = pose
        # This is the official explore_lite stop interface. It cancels its own
        # Nav2 goal before this node sends the object-approach goal.
        self.resume_pub.publish(Bool(data=False))
        self.create_timer(.5, self.try_approach, callback_group=None)
        self.say('BALL_FOUND: paused upstream explore_lite; selecting a safe stand-off pose')

    def try_approach(self):
        if self.goal_sent or self.target is None or self.map is None or not self.nav.server_is_ready(): return
        robot = self.robot_xy()
        if robot is None: return
        bx, by = self.target.pose.position.x, self.target.pose.position.y
        bearing = math.atan2(by-robot[1], bx-robot[0]); options = []
        for radius in (self.standoff, self.standoff+.20, self.standoff+.40):
            for offset in (0, .52, -.52, 1.05, -1.05, 1.57, -1.57):
                angle = bearing + offset; x = bx-radius*math.cos(angle); y = by-radius*math.sin(angle)
                if self.free(x, y): options.append((math.hypot(x-robot[0], y-robot[1]), x, y))
        if not options:
            self.say('BALL_FOUND: waiting for SLAM to map a safe stand-off pose'); return
        _, x, y = min(options)
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x; goal.pose.pose.position.y = y; goal.pose.pose.orientation = yaw_q(math.atan2(by-y, bx-x))
        self.goal_sent = True; self.say(f'APPROACHING: Nav2 goal x={x:.2f}, y={y:.2f}')
        future = self.nav.send_goal_async(goal); future.add_done_callback(self.accepted)

    def accepted(self, future):
        handle = future.result()
        if not handle.accepted: self.goal_sent = False; self.say('APPROACH_FAILED: Nav2 rejected target; waiting to retry'); return
        handle.get_result_async().add_done_callback(self.result)

    def result(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED: self.say('SUCCEEDED: reached a safe stand-off point for the red ball')
        else: self.goal_sent = False; self.say('APPROACH_FAILED: Nav2 did not reach target; waiting to retry')


def main():
    rclpy.init(); node = BallApproachManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()
