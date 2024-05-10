#!/usr/bin/python3
import rclpy
from rclpy import logging
from rclpy.node import Node, Publisher
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32

from amra_utils_py.helpers import process_yaml_input 
from amra_utils_msgs.msg import JoyIndex

class JoyActuator(Node):
    def __init__(self,):
        super().__init__("JoystickActuator") # type: ignore
        self.declare_parameter("config_path", "src/amra_utils_py/params/RhinoX56.yaml")
        self.declare_parameter("config_key", 1)
        self.declare_parameter("deadband", 0.15)
        self.deadband = self.get_parameter("deadband").get_parameter_value().double_value
        self.cb_group = ReentrantCallbackGroup()
        # Get parameters for buttons and axes
        # joystick_topic, index, dest_topic
        self.buttons, self.axes = process_yaml_input(self.get_parameter("config_path").get_parameter_value().string_value, self.get_parameter("config_key").get_parameter_value().integer_value) # type: ignore
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
        
        # Check if value is within limits
        try:
            factor = 1+self.deadband
            if (msg.data > msg.data * factor) or (msg.data < msg.data * factor):
                # Value is out of bounds, quit callback
                return
            btn["last_value"] = msg.data

        # First time through, set inital last value
        except KeyError:
            self.get_logger().warn("")
            btn.update({"last_value": msg.data})

        # Check if number is erratic, +- X% different, denoted by parameter deadband 
        btn["publisher"].publish(value)

    def commonAxsCallback(self, msg: JoyIndex):
        value = Float32()
        value.data = round(msg.data,2)*400
        for axs in self.axes:
            if axs['index'] == msg.idx:
                break

        # Check if exhaused
        if axs['index'] != msg.idx:
            self.get_logger().warning("Could not find channel index")
            return
        
        # Check if value is within limits
        try:
            # Deadband 0 < 1.0
            # 0.15
            # 0.85 -> 1.15
            exp = msg.data > ( axs["last_value"] * (1+self.deadband) )  and  msg.data < ( axs["last_value"] * (1-self.deadband) )
            exp = exp or axs["last_value"] == msg.data

            self.get_logger().debug(f"Got {exp} Value:{msg.data}; Upper: {( axs['last_value'] * (1+self.deadband) )}; Lower: { axs['last_value'] * (1-self.deadband)}")
            if ( msg.data > ( axs["last_value"] * (1+self.deadband) ) ) and ( msg.data < ( axs["last_value"] * (1-self.deadband) ) ) :
                # Value is out of bounds, quit callback
                return
            axs["last_value"] = msg.data

        # First time through, set inital last value
        except KeyError:
            self.get_logger().warn(f"Creating last_value for {msg.idx}: {msg.data}")
            axs.update({"last_value": msg.data})

        # Check if number is erratic, +- X% different, denoted by parameter deadband 
        axs["publisher"].publish(value)


def main():
    rclpy.init()
    joy_trans = JoyActuator()
    executor = MultiThreadedExecutor(8)
    executor.add_node(joy_trans)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        joy_trans.destroy_node()
    #rclpy.spin(joy_trans)
    rclpy.shutdown()

if __name__=="__main__":
    main()
        

