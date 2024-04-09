#/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class VideoStreamer(Node):
    def __init__(self):
        super().__init__("Video Streamer")
        # Todo: Open camera dith open cv
        # Todo: Make publisher
        
    
    def streamMe(self):
        #Todo: get frame 
        # Log error if failed
        
        # publish frame if successful
        pass
        

def main():
    rclpy.init()
    streamer = VideoStreamer()
    while rclpy.ok():
        streamer.streamMe()
        rclpy.spin_once(streamer)
    rclpy.shutdown()
        
    
if __name__ == "__main__":
    main()