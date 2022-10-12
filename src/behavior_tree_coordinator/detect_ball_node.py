#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters


class DetectBallNode(Node):

    def __init__(self):
        super().__init__('detect_ball_node_node')

        self.pos_publisher = self.create_publisher(PatientPosition_msg, "/patient_position/position", 1)

        human_list_subscriber = message_filters.Subscriber(self, OpenPoseHumanList, self.restamped_patient_topic)
        depth_image_subscriber = message_filters.Subscriber(self, Image, self.depth_image_topic)
        filter = message_filters.ApproximateTimeSynchronizer(
            [human_list_subscriber, depth_image_subscriber], queue_size=30, slop=3)
        filter.registerCallback(self.synchron_openpose_depth)

        self.get_logger().info("Started detect_ball_node_node")

    def synchron_openpose_depth(self, openpose_msg, depth_msg):

        # print("openpose_img:", openpose_msg.image_header.stamp.sec, ".", openpose_msg.image_header.stamp.nanosec)
        # print("depth_img:   ", depth_msg.header.stamp.sec, ".", depth_msg.header.stamp.nanosec)

        # self.counter += 1
        # print("position", self.counter)

        if openpose_msg.num_humans != 1:
            print("Unexpected num of humans")
            return

        indices = []

        request = PixelToPoint.Request()
        request.height = int(self.rgb_image_height)
        request.width = int(self.rgb_image_width)
        request.depth_image = depth_msg
        request.camera_type = self.camera_type

        # self.get_logger().info(self.camera_type)

        if self.only_centroid:
            centroid_x, centroid_y = self.get_centroid(openpose_msg.human_list[0])
            pixel = Point()
            pixel.x = float(centroid_x)
            pixel.y = float(centroid_y)
            request.pixels.append(pixel)
        else:
            for index, keypoint in enumerate(openpose_msg.human_list[0].body_key_points_with_prob):
                if keypoint.prob != 0:
                    pixel = Point()
                    pixel.x = keypoint.x
                    pixel.y = keypoint.y

                    request.pixels.append(pixel)
                    indices.append(index)

        response = self.client.call(request)

        pos_msg = PatientPosition_msg()
        pos_msg.image_header = openpose_msg.image_header

        if self.only_centroid:
            pos_msg.centroid = response.points[0]
        else:
            pos_msg.keypoints = response.points
            pos_msg.indices = indices

        self.pos_publisher.publish(pos_msg)

        # self.get_logger().info(" ".join([str(point.z) for point in response.points]))


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
