#!/usr/bin/python3

# Python Modules
from amra_utils_py.helpers import process_yaml_input
from amra_utils_msgs.msg import JoyIndex

# ROS Modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


"""
Takes in a sensor_msgs Joy message and splits up the inputs based on the topic
"""
class JoyTranslator(Node):
    def __init__(self) -> None:
        super().__init__("JoystickTranslator")
        self.declare_parameter("config_path", "src/amra_utils_py/params/RhinoX56.yaml")
        self.declare_parameter("config_key", 1)
        self.declare_parameter("joystick_in", "/joy")
        topic_name = self.get_parameter('joystick_in').get_parameter_value().string_value

        # Single param of key, from yaml master file
        # 1, 2 etc
        # use respective bindings
   
        self.buttons, self.axes = process_yaml_input(self.get_parameter("config_path").get_parameter_value().string_value, self.get_parameter("config_key").get_parameter_value().integer_value)
        self.joy_pub = self.create_subscription(Joy, topic_name, self.updateInputs, 10)

        self.createPublishers()
        self.get_logger().info("Publishing Inputs")

    """
    \breif Creats publishers and appends it to parameter
    \return None
    """
    def createPublishers(self) -> None:
        for btn in self.buttons:
            btn.update({"publisher": self.create_publisher(JoyIndex, btn["joystick_topic"], 5)})
        for axs in self.axes:
            axs.update({"publisher": self.create_publisher(JoyIndex, axs["joystick_topic"], 5)})
    
    def updateInputs(self, message: Joy) -> None:
        value = JoyIndex()
        
        for btn in self.buttons:
            channel = btn["index"]
            value.idx = channel
            value.data = message.buttons[channel]
            btn["publisher"].publish(value)

        for axs in self.axes:
            channel = axs["index"]
            value.idx = channel
            value.data = message.axes[channel]
            axs["publisher"].publish(value)
    
    """
    \return Returns a python object with keys joystick_topic, index, dest_topic
    """


def main():
    rclpy.init()
    joy_trans = JoyTranslator()
    rclpy.spin(joy_trans)
    rclpy.shutdown()

if __name__=="__main__":
    main()