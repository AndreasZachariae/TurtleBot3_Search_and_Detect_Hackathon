#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge


class DetectBallNode(Node):

    def __init__(self):
        super().__init__('detect_ball_node_node')

        self.pos_publisher = self.create_publisher(Pose, "/patient_position/position", 10)

        image_subscriber = message_filters.Subscriber(self, Image, "/intel_realsense_r200_depth/image_raw")
        depth_image_subscriber = message_filters.Subscriber(self, Image, "/intel_realsense_r200_depth/depth/image_raw")
        filter = message_filters.ApproximateTimeSynchronizer(
            [image_subscriber, depth_image_subscriber], queue_size=30, slop=3)
        filter.registerCallback(self.synchron_image_depth)

        self.bridge = CvBridge()

        self.get_logger().info("Started detect_ball_node_node")

    def synchron_image_depth(self, image_msg, depth_msg):

        # print("new image pair recieved")

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

        lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

        # Perform Otsu threshold on the A-channel
        red_thresh = cv2.threshold(lab[:, :, 1], 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # Find contours in the binary image
        contours, hierarchy = cv2.findContours(red_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            # Calculate moments for each contour
            M = cv2.moments(c)

            # Calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Mark center with small circle
            cv2.circle(cv_image, (cX, cY), 5, (0, 0, 0), -1)

        print(f"Found ball at {cX}, {cY}")

        cv2.imshow("detected ball", cv_depth_image)
        cv2.waitKey(1)

        # depth = depth_frame.get_distance(j, i)
        # depth_point = rs.rs2_deproject_pixel_to_point(
        #     depth_intrin, [j, i], depth)
        # text = "%.5lf, %.5lf, %.5lf\n" % (
        #     depth_point[0], depth_point[1], depth_point[2])


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
