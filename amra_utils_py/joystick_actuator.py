#!/usr/bin/python3
import rclpy
from rclpy import logging
from rclpy.node import Node, Publisher
from std_msgs.msg import Float32

from amra_utils_py.helpers import process_yaml_input 
from amra_utils_msgs.msg import JoyIndex

class JoyActuator(Node):
    def __init__(self,):
        super().__init__("JoystickActuator")
        self.declare_parameter("config_path", "src/amra_utils_py/params/RhinoX56.yaml")
        self.declare_parameter("config_key", 1)
        
        # Get parameters for buttons and axes
        # joystick_topic, index, dest_topic
        self.buttons, self.axes = process_yaml_input(self.get_parameter("config_path").get_parameter_value().string_value, self.get_parameter("config_key").get_parameter_value().integer_value)
        #self.get_logger().info(self.axes)

        self.createSubscribers()
        self.createPublishers()
        self.get_logger().info("Translating Joystick")
        

    def createSubscribers(self):
        #TODO: RE_WRITE SUBSCRIPTION LOGIC
        for btn in self.buttons:
            btn.update({"subscriber":self.create_subscription(JoyIndex, btn['joystick_topic'], self.commonBtnCallback, 10)})
        for axs in self.axes:
            axs.update({"subscriber":self.create_subscription(JoyIndex, axs['joystick_topic'], self.commonAxsCallback, 10)})
    
    def createPublishers(self):
        for btn in self.buttons:
            btn.update({"publisher": self.create_publisher(Float32, btn['dest_topic'], 10)})
        for axs in self.axes:
            axs.update({"publisher": self.create_publisher(Float32, axs['dest_topic'], 10)})
    
    def commonBtnCallback(self, msg: JoyIndex):
        value = Float32()

        value.data = msg.data*400

        for btn in self.buttons:
            if btn['index'] == msg.idx:
                break

        # Check if exhaused
        if btn['index'] != msg.idx:
            self.get_logger().warning("Could not find channel index")
            return
        
        btn["publisher"].publish(value)

    def commonAxsCallback(self, msg: JoyIndex):
        value = Float32()

        value.data = msg.data*400

        for axs in self.axes:
            if axs['index'] == msg.idx:
                break

        # Check if exhaused
        if axs['index'] != msg.idx:
            self.get_logger().warning("Could not find channel index")
            return

        axs["publisher"].publish(value)


def main():
    rclpy.init()
    joy_trans = JoyActuator()
    rclpy.spin(joy_trans)
    rclpy.shutdown()

if __name__=="__main__":
    main()
        

