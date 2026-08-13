#!/usr/bin/env python3
"""Detect a red ball in RGB and locate it with the aligned Gazebo PointCloud2.

False-positive hardening for the TurtleBot3 house world:
  - Higher circularity threshold (0.72) rejects elongated red furniture/cushions.
  - Larger minimum area (250 px) rejects small red specks/texture.
  - Minimum depth (0.8 m) ignores anything on the robot itself or very close.
  - Confirmation count (10) requires sustained detection across 10 frames.
  - Confirmations decay by 2 each frame the ball disappears, so a brief false
    positive does not accumulate enough to cross the threshold.
  - Exponential filter on the 3-D position is reset if a large jump is detected,
    preventing a false reading from anchoring the filtered estimate.
"""
import math
import struct

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.declare_parameter('minimum_area_px', 250.0)
        self.declare_parameter('confirmation_count', 10)
        self.declare_parameter('max_range_m', 5.0)
        self.declare_parameter('min_range_m', 0.8)       # ignore anything closer
        self.declare_parameter('min_circularity', 0.72)  # tighter than default 0.45

        self.minimum_area = self.get_parameter('minimum_area_px').value
        self.confirmations_needed = self.get_parameter('confirmation_count').value
        self.max_range = self.get_parameter('max_range_m').value
        self.min_range = self.get_parameter('min_range_m').value
        self.min_circularity = self.get_parameter('min_circularity').value

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.cloud = None
        self.confirmations = 0
        self.filtered = None
        self._published = False  # only publish marker once confirmed

        self.create_subscription(PointCloud2, '/camera/points', self.cloud_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/phase3/ball_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.status_pub = self.create_publisher(String, '/phase3/perception_status', 10)
        self.get_logger().info(
            'Ball detector ready. Waiting for camera streams. '
            f'Need {self.confirmations_needed} consecutive frames '
            f'(area>{self.minimum_area}px, circularity>{self.min_circularity}, '
            f'range {self.min_range}-{self.max_range}m).'
        )

    def cloud_cb(self, msg):
        self.cloud = msg

    @staticmethod
    def image_to_bgr(msg):
        channels = 3
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < msg.height * msg.step:
            return None
        image = raw.reshape((msg.height, msg.step))[:, :msg.width * channels]
        image = image.reshape((msg.height, msg.width, channels))
        return image if msg.encoding.lower() == 'bgr8' else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    @staticmethod
    def cloud_xyz(cloud, u, v):
        fields = {f.name: f.offset for f in cloud.fields}
        if not {'x', 'y', 'z'}.issubset(fields) or u < 0 or v < 0 or u >= cloud.width or v >= cloud.height:
            return None
        base = v * cloud.row_step + u * cloud.point_step
        order = '>' if cloud.is_bigendian else '<'
        try:
            xyz = [struct.unpack_from(order + 'f', cloud.data, base + fields[k])[0] for k in ('x', 'y', 'z')]
        except struct.error:
            return None
        return xyz if all(math.isfinite(x) for x in xyz) else None

    def image_cb(self, image_msg):
        cloud = self.cloud
        if cloud is None or cloud.header.stamp.sec == 0:
            return
        # Allow up to 3 s difference (Gazebo Classic under Docker Desktop)
        if abs((image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9) -
               (cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9)) > 3.0:
            return

        bgr = self.image_to_bgr(image_msg)
        if bgr is None:
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Both red hue wraps in HSV: [0-10] and [170-180]
        mask = (cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) |
                cv2.inRange(hsv, (170, 120, 70), (180, 255, 255)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            # Decay rather than hard reset – a single missed frame won't undo progress,
            # but a sustained absence (furniture, shadow) quickly drains the counter.
            self.confirmations = max(0, self.confirmations - 2)
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter else 0.0

        if area < self.minimum_area or circularity < self.min_circularity:
            self.confirmations = max(0, self.confirmations - 2)
            return

        m = cv2.moments(contour)
        if not m['m00']:
            return
        u, v = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])

        # Sample depth points inside the contour
        samples = []
        for yy in range(max(0, v - 10), min(cloud.height, v + 11), 3):
            for xx in range(max(0, u - 10), min(cloud.width, u + 11), 3):
                if mask[min(yy, mask.shape[0] - 1), min(xx, mask.shape[1] - 1)] == 0:
                    continue
                point = self.cloud_xyz(cloud, xx, yy)
                if point is None:
                    continue
                depth = np.linalg.norm(point)
                if self.min_range < depth < self.max_range:
                    samples.append(point)

        if not samples:
            self.confirmations = max(0, self.confirmations - 2)
            return

        xyz = np.median(np.asarray(samples), axis=0)
        depth = np.linalg.norm(xyz)

        point = PointStamped()
        point.header = cloud.header
        point.point.x, point.point.y, point.point.z = map(float, xyz)

        try:
            transform = self.tf.lookup_transform(
                'map', point.header.frame_id,
                rclpy.time.Time.from_msg(point.header.stamp),
                timeout=Duration(seconds=0.25)
            )
            mapped = do_transform_point(point, transform)
        except Exception:
            return

        observation = np.array([mapped.point.x, mapped.point.y, mapped.point.z])

        # Reset filtered estimate if the new reading is far from the current one
        # (catches cases where the first few frames were false positives)
        if self.filtered is not None and np.linalg.norm(observation - self.filtered) > 1.5:
            self.get_logger().warn(
                f'Ball position jumped {np.linalg.norm(observation - self.filtered):.2f} m '
                '– resetting filter and confirmation counter'
            )
            self.filtered = None
            self.confirmations = 0

        self.filtered = observation if self.filtered is None else 0.7 * self.filtered + 0.3 * observation
        self.confirmations += 1

        self.status_pub.publish(String(
            data=f'red ball candidate: {self.confirmations}/{self.confirmations_needed}, '
                 f'range={depth:.2f} m, '
                 f'area={area:.0f}px, circ={circularity:.2f}'
        ))

        if self.confirmations >= self.confirmations_needed:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, self.filtered)
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            self.publish_marker(pose)
            if not self._published:
                self._published = True
                self.get_logger().info(
                    f'Ball confirmed at map ({self.filtered[0]:.2f}, {self.filtered[1]:.2f}) '
                    f'after {self.confirmations} frames.'
                )

    def publish_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'phase3_ball'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.30
        marker.color.r = 1.0
        marker.color.a = 0.9
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init()
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
