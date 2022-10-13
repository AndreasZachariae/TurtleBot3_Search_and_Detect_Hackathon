#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, TransformStamped
import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DetectBallNode(Node):

    def __init__(self):
        super().__init__('detect_ball_node_node')

        self.set_parameters([Parameter(name="use_sim_time", value=True)])

        self.ball_pose_publisher = self.create_publisher(Pose, "/ball_pose", 10)
        self.depth_camera_info = self.create_subscription(
            CameraInfo, "/intel_realsense_r200_depth/depth/camera_info", self.depth_camera_info_callback, 10)

        image_subscriber = message_filters.Subscriber(self, Image, "/intel_realsense_r200_depth/image_raw")
        depth_image_subscriber = message_filters.Subscriber(self, Image, "/intel_realsense_r200_depth/depth/image_raw")
        filter = message_filters.ApproximateTimeSynchronizer(
            [image_subscriber, depth_image_subscriber], queue_size=30, slop=3)
        filter.registerCallback(self.synchron_image_depth)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_intrinsics = rs.intrinsics()

        self.get_logger().info("Started detect_ball_node_node")

    def depth_camera_info_callback(self, info_msg):
        self.depth_intrinsics.width = info_msg.width
        self.depth_intrinsics.height = info_msg.height
        self.depth_intrinsics.ppx = info_msg.k[2]
        self.depth_intrinsics.ppy = info_msg.k[5]
        self.depth_intrinsics.fx = info_msg.k[0]
        self.depth_intrinsics.fy = info_msg.k[4]
        # self.depth_intrinsics.model = info_msg.distortion_model
        self.depth_intrinsics.model = rs.distortion.none
        self.depth_intrinsics.coeffs = [i for i in info_msg.d]

        self.destroy_subscription(self.depth_camera_info)

    def synchron_image_depth(self, image_msg, depth_msg):

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

        # Normalize image to 0-1 values
        normalized_image = cv_image/255

        # Calculate red chromaticity
        red_chromaticity_image = normalized_image[:, :, 2] / \
            (normalized_image[:, :, 0] + normalized_image[:, :, 1] + normalized_image[:, :, 2])

        # threshold for red ball
        red_mask = np.uint8(red_chromaticity_image*255)
        red_mask = cv2.threshold(red_mask, 175, 255, cv2.THRESH_BINARY)[1]

        # Find contours in the binary image
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 1:
            # self.get_logger().info("none or too many red blob(s) detected, passing...")
            return

        # Calculate moments for contour
        M = cv2.moments(contours[0])

        # Calculate x,y coordinate of center
        if M["m00"] == 0:
            self.get_logger().info("devision by zero, passing...")
            return
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        if cX >= image_msg.height:
            cX = image_msg.height-1
        if cY >= image_msg.width:
            cY = image_msg.width-1

        # Mark center with small circle and display
        # cv2.circle(cv_depth_image, (cX, cY), 5, (0, 0, 0), 2)
        # cv2.imshow("detected ball", cv_depth_image)
        # cv2.waitKey(1)

        # Get depth value on ball pixel position and deproject to 3D coords
        depth = cv_depth_image[cX, cY]
        depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [cX, cY], depth)

        if any([depth == float("inf"), depth_point[0] == float("inf"), depth_point[1] == float("inf"), depth_point[2] == float("inf")]):
            self.get_logger().info("none or infinity value at depth image, passing...")
            return

        print(f"Found ball at {cX}, {cY}")
        print(f"depth = {depth}")
        print("point = %.5lf, %.5lf, %.5lf\n" % (depth_point[0], depth_point[1], depth_point[2]))

        # Publish transform from "camera_depth_frame" to visualize in RVIZ and for later transform
        t = TransformStamped()
        t.header.stamp.sec = depth_msg.header.stamp.sec  # -5
        t.header.stamp.nanosec = depth_msg.header.stamp.nanosec
        t.header.frame_id = "camera_depth_frame"
        t.child_frame_id = "ball"
        t.transform.translation.x = depth_point[0]
        t.transform.translation.y = depth_point[1]
        t.transform.translation.z = depth_point[2]
        self.tf_broadcaster.sendTransform(t)

        # Transform Ball relative to "map" frame
        try:
            ball_tf = self.tf_buffer.lookup_transform(
                "map",
                "ball",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform 'map' to 'ball': {ex}")
            return

        # Publish ball_pose to topic in the Behavior Tree
        ball_pose = Pose()
        ball_pose.position.x = ball_tf.transform.translation.x
        ball_pose.position.y = ball_tf.transform.translation.y
        ball_pose.position.z = ball_tf.transform.translation.z
        ball_pose.orientation.x = ball_tf.transform.rotation.x
        ball_pose.orientation.y = ball_tf.transform.rotation.y
        ball_pose.orientation.z = ball_tf.transform.rotation.z
        ball_pose.orientation.w = ball_tf.transform.rotation.w
        self.ball_pose_publisher.publish(ball_pose)


def main(args=None):

    rclpy.init(args=args)

    node = DetectBallNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
