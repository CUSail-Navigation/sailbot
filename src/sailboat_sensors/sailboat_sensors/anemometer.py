import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
#from Adafruit_ADS1x15 import ADS1x15 as ADS
import serial

class Anemometer(Node):

    def __init__(self):
        super().__init__('anemometer')

        # Declare timer period parameter
        self.declare_parameter('timer_period', 0.5)
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('anemometer_port')
        self.adc_address = self.get_parameter('anemometer_port').value

        self.publisher_ = self.create_publisher(Int32, '/wind', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.anem_serial = serial.Serial(self.adc_address, baudrate=9600)
        self.get_logger().info('Launching Anemometer')


    def read_wind(self):

        try:
            if self.anem_serial.in_waiting > 0:
                # Read the incoming message from XBee
                wind_dir = self.serial_port.readline().decode('utf-8').strip()
                return int(wind_dir)

        except Exception as e:
            self.get_logger().error(f"Error reading anem data: {e}")

    def timer_callback(self):
        msg = Int32()
        msg.data = self.read_wind()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Wind Angle: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    anem = Anemometer()

    rclpy.spin(anem)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    anem.destroy_node()
    rclpy.shutdown()

