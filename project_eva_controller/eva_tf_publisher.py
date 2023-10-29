#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
import math
import numpy as np

# Function to calculate quaternion from Euler angles


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class MotorTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('motor_transform_broadcaster')

        # Initialize the TransformBroadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize variables for distance and rotation
        self.distance_x = 0
        self.distance_y = 0
        self.rotation_z = 0

        # Create a subscription to the Joy topic
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # Get the joystick axes values
        axes = msg.axes

        # Update distance and rotation based on joystick input
        self.distance_x += (axes[1] * math.cos(self.rotation_z)
                            * 0.03)+(axes[0] * math.sin(self.rotation_z) * 0.03)
        self.distance_y += (axes[1] * math.sin(self.rotation_z)
                            * 0.03) + (axes[0] * math.cos(self.rotation_z) * 0.03)

        # self.distance_y += axes[0] * 0.035
        self.rotation_z += axes[3] * 0.07

        # Create a TransformStamped message
        t = TransformStamped()

        # Set the header values for the message
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # Set the translation values for the message
        t.transform.translation.x = self.distance_x
        t.transform.translation.y = self.distance_y
        t.transform.translation.z = 0.0

        # Calculate the quaternion from Euler angles for rotation
        q = quaternion_from_euler(0, 0, self.rotation_z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MotorTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
