#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from point_transformation_interfaces.srv import PixelToPoint


class Client():

    def __init__(self, node: Node):

        self.node = node

        self.client = self.node.create_client(
            PixelToPoint, 'point_transformation_node/pixel_to_point')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('PixelToPoint service not available, waiting again...')

        self.request = PixelToPoint.Request()

    def pixel_to_point_async(self, pixel, height=0, width=0, depth_image:Image=Image()):

        self.request.pixel.x = float(pixel[0])
        self.request.pixel.y = float(pixel[1])
        self.request.height = int(height)
        self.request.width = int(width)
        self.request.depth_image = depth_image

        print(str(depth_image))

        return self.client.call_async(self.request)

    def pixel_to_point(self, pixel, height=0, width=0, depth_image:Image=Image()):

        future = self.pixel_to_point_async(pixel, height, width, depth_image)

        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()

        self.node.get_logger().info(str(response.point))

        return response.point


def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('example_node')

    ### Usage example ###

    client = Client(node)

    # Image collected via camera driver, e.g. rc_visard_ros
    depth_image = Image(height=640, width=480, encoding="rgb8")

    # Pixel to be transformed to 3D point
    pixel = [100, 100]
    rgb_image_size = [1200, 900]

    point = client.pixel_to_point(pixel, rgb_image_size[0], rgb_image_size[1], depth_image)

    #####################

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
