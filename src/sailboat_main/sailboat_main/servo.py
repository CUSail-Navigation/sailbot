import numpy as np
import rclpy
import serial
from rclpy.node import Node

from sailboat_interface import SailTail

class Servo(Node):
    def __init__(self):
        super().__init__('servo')

        self.declare_parameter('servo_port')
        self.servo_port = self.get_parameter('servo_port').value

        self.subscription = self.create_subscription(
            SailTail,
            'servomux',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.servo_serial = serial.Serial(self.servo_port, baudrate=9600)

    def listener_callback(self, msg):
        self.servo_serial.write(str(msg.sail) + ' ' + str(msg.tail))
        self.get_logger().info('Sail:' + str(msg.sail) + ' Tail:' + str(msg.tail))
 
def main(args=None):
    rclpy.init(args=args)

    servo = Servo()

    rclpy.spin(servo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    airmar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

