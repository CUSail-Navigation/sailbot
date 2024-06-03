import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from Adafruit_ADS1x15 import ADS1x15 as ADS

from sailboat_sensors.sailboat_sensors import ADCDevice


class ReadWind(Node):

    def __init__(self):
        super().__init__('ReadWind')

        # Declare timer period parameter
        self.declare_parameter('timer_period', 1.0)
        self.timer_period = self.get_parameter('timer_period').value

        self.publisher_ = self.create_publisher(Int32, 'wind', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.sail_anemometer = ADCDevice(0)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.sail_anemometer.readWind()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Wind Angle: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    read_wind = ReadWind()

    rclpy.spin(read_wind)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_wind.destroy_node()
    rclpy.shutdown()

