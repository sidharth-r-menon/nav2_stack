#!/usr/bin/env python3
"""
apriltag_dock_detector.py
Real-time AprilTag 36h11 detector & dock pose publisher for OpenNav Docking in ROS 2 Jazzy.
Subscribes to /camera/image_raw, detects Tag ID 0, broadcasts TF 'dock_tag',
and publishes geometry_msgs/PoseStamped on /detected_dock_pose for SimpleChargingDock.
"""

import math
import numpy as np
import cv2
import pupil_apriltags
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import tf2_ros


class AprilTagDockDetector(Node):
    def __init__(self):
        super().__init__('apriltag_dock_detector')
        
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_size', 0.15)  # 15 cm tag
        self.declare_parameter('camera_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('dock_frame', 'dock_tag')
        self.declare_parameter('output_frame', 'odom')
        
        self.target_tag_id = int(self.get_parameter('tag_id').value)
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.dock_frame = str(self.get_parameter('dock_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        
        self.bridge = CvBridge()
        self.detector = pupil_apriltags.Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )
        
        # Default intrinsics for TurtleBot3 Waffle Pi camera (640x480)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_info_received = False
        
        # TF Broadcaster & Buffer
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers & Subscribers
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.debug_img_pub = self.create_publisher(Image, '/camera/dock_detection', 10)
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
            
        self.get_logger().info(f'AprilTag Dock Detector active for Tag ID {self.target_tag_id} (size: {self.tag_size:.2f}m)')

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received and msg.k[0] > 0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
            self.camera_info_received = True
            self.get_logger().info(f'Camera intrinsics loaded: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        camera_params = [self.fx, self.fy, self.cx, self.cy]
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=self.tag_size
        )
        
        target_det = None
        for det in detections:
            if det.tag_id == self.target_tag_id:
                target_det = det
                break
                
        if target_det is not None:
            t_cam = target_det.pose_t.ravel() # [tx, ty, tz]
            r_mat = target_det.pose_R # 3x3 rotation matrix
            
            # Scipy rotation to quaternion [qx, qy, qz, qw]
            rot = Rotation.from_matrix(r_mat)
            quat = rot.as_quat()
            
            # Broadcast TF: camera_rgb_optical_frame -> dock_tag
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.camera_frame
            tf_msg.child_frame_id = self.dock_frame
            tf_msg.transform.translation.x = float(t_cam[0])
            tf_msg.transform.translation.y = float(t_cam[1])
            tf_msg.transform.translation.z = float(t_cam[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf_msg)
            
            # Transform detected dock pose into odom frame for OpenNav Docking
            try:
                if self.tf_buffer.can_transform(self.output_frame, self.camera_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.05)):
                    t_odom_cam = self.tf_buffer.lookup_transform(self.output_frame, self.camera_frame, rclpy.time.Time())
                    
                    t_cam_tag = np.eye(4)
                    t_cam_tag[:3, :3] = r_mat
                    t_cam_tag[:3, 3] = t_cam
                    
                    tx = t_odom_cam.transform.translation.x
                    ty = t_odom_cam.transform.translation.y
                    tz = t_odom_cam.transform.translation.z
                    oq = [
                        t_odom_cam.transform.rotation.x,
                        t_odom_cam.transform.rotation.y,
                        t_odom_cam.transform.rotation.z,
                        t_odom_cam.transform.rotation.w
                    ]
                    m_odom_cam = np.eye(4)
                    m_odom_cam[:3, :3] = Rotation.from_quat(oq).as_matrix()
                    m_odom_cam[:3, 3] = [tx, ty, tz]
                    
                    m_odom_tag = np.dot(m_odom_cam, t_cam_tag)
                    q_odom_tag = Rotation.from_matrix(m_odom_tag[:3, :3]).as_quat()
                    
                    dock_pose_msg = PoseStamped()
                    dock_pose_msg.header.stamp = self.get_clock().now().to_msg()
                    dock_pose_msg.header.frame_id = self.output_frame
                    dock_pose_msg.pose.position.x = float(m_odom_tag[0, 3])
                    dock_pose_msg.pose.position.y = float(m_odom_tag[1, 3])
                    dock_pose_msg.pose.position.z = float(m_odom_tag[2, 3])
                    dock_pose_msg.pose.orientation.x = float(q_odom_tag[0])
                    dock_pose_msg.pose.orientation.y = float(q_odom_tag[1])
                    dock_pose_msg.pose.orientation.z = float(q_odom_tag[2])
                    dock_pose_msg.pose.orientation.w = float(q_odom_tag[3])
                    self.pose_pub.publish(dock_pose_msg)
            except Exception:
                dock_pose_msg = PoseStamped()
                dock_pose_msg.header.stamp = self.get_clock().now().to_msg()
                dock_pose_msg.header.frame_id = self.camera_frame
                dock_pose_msg.pose.position.x = float(t_cam[0])
                dock_pose_msg.pose.position.y = float(t_cam[1])
                dock_pose_msg.pose.position.z = float(t_cam[2])
                dock_pose_msg.pose.orientation.x = float(quat[0])
                dock_pose_msg.pose.orientation.y = float(quat[1])
                dock_pose_msg.pose.orientation.z = float(quat[2])
                dock_pose_msg.pose.orientation.w = float(quat[3])
                self.pose_pub.publish(dock_pose_msg)

            # Draw overlay
            corners = np.int32(target_det.corners)
            cv2.polylines(cv_image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.circle(cv_image, tuple(np.int32(target_det.center)), 4, (0, 0, 255), -1)
            dist = math.sqrt(t_cam[0]**2 + t_cam[1]**2 + t_cam[2]**2)
            cv2.putText(cv_image, f"Dock Tag ID 0: {dist:.2f}m", (corners[0][0], corners[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
        try:
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
