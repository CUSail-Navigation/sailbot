import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from . import licor_real
from . import licor_fake


class LicorNode(Node):
    def __init__(self):
        super().__init__('licor')

        self.declare_parameter('timer_period', 0.1)
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('licor_port', '/dev/ttyUSB0')
        self.licor_port = self.get_parameter('licor_port').value

        self.declare_parameter('baud_rate', 115200)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('use_fake_data', False)
        self.use_fake_data = self.get_parameter('use_fake_data').value

        self.declare_parameter('min_speed', 1.0)
        self.min_speed = self.get_parameter('min_speed').value

        # EMA smoothing coefficient: 0 = frozen, 1 = no filtering
        self.declare_parameter('alpha', 0.3)
        self.alpha = self.get_parameter('alpha').value

        self.publisher = self.create_publisher(Int32, 'wind', 10)
        self.timer = self.create_timer(self.timer_period, self.publish_wind)
        self.last_direction = 0
        self.filtered_u = 0.0  # sin component
        self.filtered_v = 1.0  # cos component

        if self.use_fake_data:
            self.sensor = licor_fake.FakeLicorAnemometer()
            self.get_logger().info('Using fake data for LI-COR anemometer')
        else:
            self.sensor = licor_real.LicorAnemometer(self.licor_port, self.baud_rate)
            self.get_logger().info(f'LI-COR anemometer on {self.licor_port} at {self.baud_rate} baud')

    def publish_wind(self):
        speed = self.sensor.read_speed()
        if speed >= self.min_speed:
            raw = math.radians(self.sensor.read_direction())
            self.filtered_u += self.alpha * (math.sin(raw) - self.filtered_u)
            self.filtered_v += self.alpha * (math.cos(raw) - self.filtered_v)
            self.last_direction = int(math.degrees(math.atan2(self.filtered_u, self.filtered_v)) % 360)
        msg = Int32()
        msg.data = self.last_direction
        self.publisher.publish(msg)
        self.get_logger().info(f'Wind direction: {msg.data} (speed: {speed:.2f} m/s)')


def main(args=None):
    rclpy.init(args=args)
    node = LicorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
