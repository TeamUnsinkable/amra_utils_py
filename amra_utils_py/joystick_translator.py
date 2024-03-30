#!/usr/bin/bash

import rclpy
import os
import json
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
        self.declare_parameter("device", 0)

        topic_name = self.process_yaml_input(self.get_parameter("config_path").get_parameter_value()._string_value)
        self.joy_pub = self.create_subscription(Joy, topic_name, self.updateInputs, 10)

        self.createPublishers()
        print()

    """
    \breif Creats publishers and appends it to parameter
    \return None
    """
    def createPublishers(self) -> None:
        for btn in self.buttons:
            topic_name = btn["data"]["topic"]
            btn["data"]["publisher"] = self.create_publisher(Float32, topic_name, 5)

        for axs in self.axes:
            topic_name = axs["data"]["topic"]
            axs["data"]["publisher"] = self.create_publisher(Float32, topic_name, 5)
    
    def updateInputs(self, message: Joy) -> None:
        
        for btn in self.buttons:
            channel = btn["data"]["index"]
            value = Float32()
            value.data = message.buttons[channel]
            btn["data"]["publisher"].publish(Float32(value))

        for axs in self.axes:
            channel = axs["data"]["index"]
            value = Float32()
            value.data = message.axes[channel]
            axs["data"]["publisher"].publish(value)
    
    def process_yaml_input(self, yaml_data_path):
        self.buttons = []
        self.axes = []
        print(os.getcwd())
        print(os.path.join(os.getcwd(), yaml_data_path))
        #TODO: Fix path completion
        with open(os.path.join(os.getcwd(), yaml_data_path), 'r') as book:
            yaml_data = book.read()
      
        yaml_data = json.loads(yaml_data)

        for id, attributes in yaml_data.items():
            data = {'index': attributes['index'], 'topic': attributes['topic']}
            entry = {"topic_in": id, "data": data}

            if attributes['type'] == 0:
                self.buttons.append(entry)
            elif attributes['type'] == 1:
                self.axes.append(entry)
            
        return id

def main():
    rclpy.init()
    joy_trans = JoyTranslator()
    rclpy.spin(joy_trans)
    rclpy.shutdown()

if __name__=="__main__":
    main()