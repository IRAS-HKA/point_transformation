#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthImageSub(Node):
    # not used, just for testing

    def __init__(self):
        super().__init__('depth_image_sub')

        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(Image, '/stereo/depth', self.image_callback, 10)

        self.get_logger().info("Started depth_image_sub")

    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

        print("image: "+msg.encoding + str(msg.width) + str(msg.height))

        cv2.patchNaNs(cv_image, val=0)

        for i in range(640):
            if not np.isnan(cv_image[0, i]):
                print(str(i) + ", 0) " + str(cv_image[0, i]))


def main(args=None):

    rclpy.init(args=args)

    node = DepthImageSub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
