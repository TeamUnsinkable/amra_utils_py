#!/usr/bin/python3
import rclpy
from rclpy import logging
from rclpy.node import Node, Publisher
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from threading import Lock
from time import sleep
from amra_utils_py.helpers import process_yaml_input 
from amra_utils_msgs.msg import JoyIndex

class JoyActuator(Node):
    def __init__(self,):
        super().__init__("JoystickActuator") # type: ignore
        self.declare_parameter("config_path", "src/amra_utils_py/params/RhinoX56.yaml")
        self.declare_parameter("config_key", 1)
        self.declare_parameter("deadband", 0.15)
        self.declare_parameter("scaling_factor", 400.0)
        self.deadband = self.get_parameter("deadband").get_parameter_value().double_value
        self.scaling_factor = self.get_parameter("scaling_factor").get_parameter_value().double_value
        self.cb_group = ReentrantCallbackGroup()
        # Get parameters for buttons and axes
        # joystick_topic, index, dest_topic
        self.buttons, self.axes = process_yaml_input(self.get_parameter("config_path").get_parameter_value().string_value, self.get_parameter("config_key").get_parameter_value().integer_value) # type: ignore
        #self.get_logger().info(self.axes)
        self.res_lock = Lock()
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
        self.axs_pubs = {}
        self.btn_pubs = {}
        for btn in self.buttons:
            self.btn_pubs.update( {btn["index"]: {"publisher": self.create_publisher(Float64, btn['dest_topic'], 10)} } )
        for axs in self.axes:
            self.axs_pubs.update( {axs["index"]: {"publisher": self.create_publisher(Float64, axs['dest_topic'], 10)} } )
        #self._logger.warn(self.axs_pubs)
    
    def commonBtnCallback(self, msg: JoyIndex):
        value = Float64()
        value.data = msg.data
        
        self.get_logger().info(f"Got to btn:{msg.idx} with val:{msg.data}")

        try:
            btn = self.btn_pubs[msg.idx]
        except KeyError:
            self.get_logger().warning("Could not find channel index")
            return
        
        # Check if value is the last
        try:
            if btn["last_value"] == msg.data:
                return
            btn["last_value"] = msg.data
        # First time through, set inital last value
        except KeyError:
            self.get_logger().warn("")
            btn.update({"last_value": msg.data})
        
        # Publish value
        btn["publisher"].publish(value)

    def commonAxsCallback(self, msg: JoyIndex):
        value = Float64()
        value.data = msg.data*self.scaling_factor
        
        try:
            axs = self.axs_pubs[msg.idx]
            #self._logger.warn(f"Publishing {msg.idx}: from: {msg.data} got: {msg.data*self.scaling_factor}")
        except KeyError:
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
                self._logger.debug(f"Axis {msg.idx} is out of margin")
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
        

