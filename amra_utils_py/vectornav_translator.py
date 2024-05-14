#!/usr/bin/bash

import rclpy
from rclpy.node import Node
from vectornav_msgs.msg import CommonGroup
from std_msgs.msg import Float64
from rclpy.callback_groups import ReentrantCallbackGroup

class VectornavTranslator(Node):
    def __init__(self):
        super().__init__('VectornavTranslator')
        self.cb_group = ReentrantCallbackGroup()
        self.common_x_pub = self.create_publisher(Float64, "vectornav/translated/attitude/x", 10, callback_group=self.cb_group)
        self.common_y_pub = self.create_publisher(Float64, "vectornav/translated/attitude/y", 10, callback_group=self.cb_group)
        self.common_z_pub = self.create_publisher(Float64, "vectornav/translated/attitude/z", 10, callback_group=self.cb_group)

        self.create_subscription(CommonGroup, "vectornav/raw/common", self.recv_common, 10)
        self.get_logger().warning("Spun up and spitting messages")

    def recv_common(self, message: CommonGroup) -> None:
        value = Float64()

        # Z Data
        value.data = message.yawpitchroll.z
        self.common_z_pub.publish(value)

        # Y Data
        value.data = message.yawpitchroll.y
        self.common_y_pub.publish(value)

        # Z Data01
        value.data = message.yawpitchroll.x
        self.common_x_pub.publish(value)
    
    
def main():
    rclpy.init()
    trans = VectornavTranslator()
    rclpy.spin(trans)
    rclpy.shutdown()

if __name__=="__main__":
    main()
