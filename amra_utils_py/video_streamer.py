#/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np



class VideoStreamer(Node):
    def __init__(self):
        super().__init__("Video_Streamer")
        # Todo: Open camera with* open cv
        #STOOOPID !!!!!
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
        cv2.imshow("ASDFGHASDFGH", frame)
        # Todo: Make publisher
        
    #invalid :(
    def streamMe(self):
        #Todo: get frame
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        edges = cv2.Canny(hsv, 50, 150)
        # Log error if failed
        if not ret:
            print("no worky :( bwomp)")
            pass
        # publish frame if successful
        cv2.imshow("wahoo", edges)
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