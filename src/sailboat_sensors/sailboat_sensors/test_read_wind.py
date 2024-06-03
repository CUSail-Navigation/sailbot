import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

"""
Basic read wind test. This can be modified later to randomize wind angle for testing purposes.
"""
class TestReadWind(Node):
    def __init__(self):
        super().__init__('test_read_wind')
        self.publisher_ = self.create_publisher(Int32, 'wind', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = 12
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Wind Angle: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    test_read_wind = TestReadWind()

    rclpy.spin(test_read_wind)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_read_wind.destroy_node()
    rclpy.shutdown()
