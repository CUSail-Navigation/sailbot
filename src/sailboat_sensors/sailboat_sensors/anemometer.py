import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from Adafruit_ADS1x15 import ADS1x15 as ADS

class Anemometer(Node):

    def __init__(self):
        super().__init__('anemometer')

        # Declare timer period parameter
        self.declare_parameter('timer_period', 0.5)
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('ADC_ADDRESS', '0x48')
        self.adc_address = self.get_parameter('ADC_ADDRESS').value

        self.publisher_ = self.create_publisher(Int32, '/wind', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.mainADC = ADS.ADS1015(self.adc_address)
        self.get_logger().info('Launching Anemometer')


    def read_wind(self):
        rawadc = self.mainADC.read_adc(0, gain=2/3) 
        wind_direction = (180 + 360 - rawadc * 360 / 1720) % 360
        return int(wind_direction)

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

