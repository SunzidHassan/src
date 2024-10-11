#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from functools import partial
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import numpy as np
import cv2 as cv
import sensor_msgs_py.point_cloud2 as pc2
import sys

class TurtleControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("turtle_controller")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)
        self.image_subscribe = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10)
        self.pointCloud_subscirbe = self.create_subscription(PointCloud2, "/camera/points", self.pointCloud_callback, 10)
        self.get_logger().info("Turtle controller node has been started")
        self.get_logger().info(str(sys.executable))

        self.grid_size = (500, 500)
        self.grid_map = np.zeros(self.grid_size, dtype=np.uint8)

    def tf_callback(self, msg: TFMessage):
        if msg.transforms[0].header.frame_id == "odom": pass
            # self.get_logger().info("x: " + str(msg.transforms[0].transform.translation.x)\
            #                         +"\n" + "y: " + str(msg.transforms[0].transform.translation.y)\
            #                         +"\n" + "z: " + str(msg.transforms[0].transform.rotation.z)\
            #                         +"\n" + "w: " + str(msg.transforms[0].transform.rotation.w))

    def image_callback(self, msg: CompressedImage): pass
        # picTime = int(msg.header.stamp.sec)
        # self.get_logger().info("time: " + str(picTime))
        # if picTime % 10 == 0:
        #     image = np.asarray(bytearray(msg.data), dtype="uint8")
        #     self.imgDecode = cv.imdecode(image, cv.IMREAD_COLOR)
        #     cv.imwrite(f'/my_bot/src/my_robot_controller/my_robot_controller/temp/{picTime}.png', self.imgDecode)

    def pointCloud_callback(self, msg: PointCloud2):
        pointTime = int(msg.header.stamp.sec)
        self.get_logger().info("Time: " + str(pointTime))
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.generate_grid_map(points)
        # for point in points:
        #     self.get_logger().info(f'Point: {point}')

    def generate_grid_map(self, points):
        self.grid_map.fill(0)

        grid_resolution = 0.1 # meters per cell
        max_x, max_y = self.grid_size[0] * grid_resolution / 2, self.grid_size[1] * grid_resolution / 2

        for point in points:
            x, y, z = point
            if -max_x <= x <= max_x and -max_y <= y <= max_y:
                grid_x = int((x + max_x) / grid_resolution)
                grid_y = int((y + max_y) / grid_resolution)
                self.grid_map[grid_y, grid_x] = 255

        # Display the grid map
        cv.imwrite(f'/my_bot/src/my_robot_controller/my_robot_controller/temp/grid_map.png', self.grid_map)


    def send_velocity_command(self):
        msg = Twist() # create a msg object from Twist() class
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()