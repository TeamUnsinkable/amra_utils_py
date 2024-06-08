import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

class ArduinoSerialSender(Node):

    def __init__(self):
        super().__init__('Arduino_serial_sender')
        self.subscription_direction = self.create_subscription(
            Float64,
            '/controls/arm/jawDir',
            self.listener_callback_direction,
            10)
        self.subscription_movement = self.create_subscription(
            Float64,
            '/controls/arm/jawMove',
            self.listener_callback_movement,
            10)
        
        # self.serial_port = serial.Serial('/dev/ttyACM3', 115200)
        self.direction = 0
        self.movement = 0
        self.get_logger().info("Arduino Serial node now screaming at Arduino.")
    def listener_callback_direction(self, msg):
        self.direction = msg.data
        self.send_to_arduino()

    def listener_callback_movement(self, msg):
        self.movement = msg.data
        self.send_to_arduino()

    def send_to_arduino(self):
        message = f'{self.direction},{self.movement}\n'
       # self.serial_port.write(message.encode())
        self.get_logger().info(f'Sent {message}')

def main(args=None):
    rclpy.init(args=args)
    serial_sender = ArduinoSerialSender()
    rclpy.spin(serial_sender)
    serial_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
