#/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class VideoReciever(Node):
    def __init__(self):
        super().__init__("VideoViewer")
        # Todo: create display window, might need to create class variable
        # Todo: Make subscriber with callback to updateDisplay
        
    
    def updateDisplay(self, msg: Image):
        # Todo: update display window
        pass
        

def main():
    rclpy.init()
    viewer = VideoReciever()
    rclpy.spin(viewer)
    rclpy.shutdown()
        
    
if __name__ == "__main__":
    main()