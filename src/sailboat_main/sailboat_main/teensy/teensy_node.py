import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8
from .. import constants
from . import teensy
from . import teensy_fake


class Teensy(Node):
    """
    ROS 2 Node that interfaces with a Teensy microcontroller to control and
    monitor the Sailbot's hardware: mainsail, rudder, and jib. It communicates
    with the Teensy in real or simulated mode, reads telemetry (wind, angles,
    dropped packets), and sends control commands from ROS subscriptions.

    Command topics (namespace-relative): ``sail`` (Int32), ``rudder_angle`` (Int32),
    ``jib_angle`` (Int32), ``jib_side_flag`` (UInt8: 0 = port side, 1 = starboard side).
    """

    def __init__(self):
        super().__init__('teensy')

        # Declare parameters.
        self.declare_parameter('teensy_port', '/dev/ttyACM0')
        self.declare_parameter('simulated', False)

        # Poll telemetry on ``rx_period`` (seconds — ``create_timer`` does not use ms). Default is
        # ``SERIAL.RX_PERIOD_MS / 1000`` so the host matches the Teensy TX period in ``constants``.
        self.declare_parameter('rx_period', constants.SERIAL.RX_PERIOD_MS / 1000.0)

        # Get parameters.
        self.timer_period = float(self.get_parameter('rx_period').value)
        self.teensy_port = self.get_parameter('teensy_port').value
        self.simulated = self.get_parameter('simulated').value

        # Initialize the appropriate servo handler based on the simulation mode.
        if self.simulated:
            self.teensy = teensy_fake.TeensyFake()
            self.get_logger().info('Simulation mode enabled. Serial communication is disabled.')
        else:
            self.teensy = teensy.TeensyHardware(self.teensy_port)
            self.get_logger().info(f'Real mode enabled. Serial communication on {self.teensy_port}.')

        # Keep track of our desired mainsail, rudder, and jib angles.
        self.desired_mainsail_angle = 0
        self.desired_rudder_angle = 0
        self.desired_jib_angle = 0
        self.desired_jib_side_flag = 0

        # Telemetry data publishers.
        self.wind_angle_pub = self.create_publisher(Int32, 'wind', 10)
        self.actual_mainsail_angle_pub = self.create_publisher(Int32, 'actual_sail_angle', 10)
        self.actual_rudder_angle_pub = self.create_publisher(Int32, 'actual_rudder_angle', 10)
        self.actual_jib_angle_pub = self.create_publisher(Int32, 'actual_jib_angle', 10)
        self.actual_jib_side_flag_pub = self.create_publisher(UInt8, 'actual_jib_side_flag', 10)
        self.dropped_packets_pub = self.create_publisher(Int32, 'dropped_packets', 10)

        # Callback to read Teensy data.
        self.timer = self.create_timer(self.timer_period, self.check_telemetry)
        self.create_subscription(Int32, 'sail', self.mainsail_angle_callback, 10)
        self.create_subscription(Int32, 'rudder_angle', self.rudder_angle_callback, 10)
        self.create_subscription(Int32, 'jib_angle', self.jib_angle_callback, 10)
        self.create_subscription(UInt8, 'jib_side_flag', self.jib_side_flag_callback, 10)

    def check_telemetry(self):
        """
        Timer based function that checks for Teensy telemetry data. Publishes
        data to telemetry topics.
        """
        self.get_logger().info('Check telemetry callback entered.')
        data = {}
        if self.teensy.read_telemetry(data) == 0:
            wind_angle_msg = Int32()
            wind_angle_msg.data = data['wind_angle']
            self.wind_angle_pub.publish(wind_angle_msg)

            mainsail_angle_msg = Int32()
            mainsail_angle_msg.data = data['mainsail_angle']
            self.actual_mainsail_angle_pub.publish(mainsail_angle_msg)

            rudder_angle_msg = Int32()
            rudder_angle_msg.data = data['rudder_angle']
            self.actual_rudder_angle_pub.publish(rudder_angle_msg)

            jib_angle_msg = Int32()
            jib_angle_msg.data = data['jib_angle']
            self.actual_jib_angle_pub.publish(jib_angle_msg)

            jib_side_flag_msg = UInt8()
            jib_side_flag_msg.data = data['jib_side_flag']
            self.actual_jib_side_flag_pub.publish(jib_side_flag_msg)

            dropped_packets_msg = Int32()
            dropped_packets_msg.data = data['dropped_packets']
            self.dropped_packets_pub.publish(dropped_packets_msg)

            # Write rudder angle to file in append mode.
            try:
                file_path = '/home/ros2_user/ros2_ws/src/log/' + 'rudder_angle_log.txt'
                self.get_logger().info(f'Writing rudder angle to file {file_path}')
                with open(file_path, 'a') as f:
                    f.write(f'{rudder_angle_msg.data}\n')
            except Exception as e:
                self.get_logger().warn(f'Failed to write rudder angle to file: {e}')

            self.get_logger().info(f"{'Wind angle:':<20} {wind_angle_msg.data}")
            self.get_logger().info(f"{'Actual mainsail angle:':<20} {mainsail_angle_msg.data}")
            self.get_logger().info(f"{'Actual rudder angle:':<20} {rudder_angle_msg.data}")
            self.get_logger().info(f"{'Actual jib angle:':<20} {jib_angle_msg.data}")
            self.get_logger().info(f"{'Actual jib side flag:':<20} {jib_side_flag_msg.data}")
            self.get_logger().info(f"{'Dropped packets:':<20} {dropped_packets_msg.data}")
        else:
            self.get_logger().info('No telemetry received.')

    def _send_command_to_teensy(self):
        """
        Send the latest mainsail, rudder, and jib goals in one serial packet.
        """
        if self.teensy.send_command(
                self.desired_mainsail_angle,
                self.desired_rudder_angle,
                self.desired_jib_angle,
                self.desired_jib_side_flag,
        ) == 0:
            self.get_logger().info(
                f'Message sent to Teensy: mainsail:{self.desired_mainsail_angle}, rudder:{self.desired_rudder_angle} '
                f'jib:{self.desired_jib_angle}, jib side:{self.desired_jib_side_flag}'
            )
        else:
            self.get_logger().warn('Message failed to send to Teensy.')

    def mainsail_angle_callback(self, msg):
        """ Handle ``sail`` updates and forward the full command packet. """
        self.desired_mainsail_angle = msg.data
        self._send_command_to_teensy()

    def rudder_angle_callback(self, msg):
        """ Handle ``rudder_angle`` updates and forward the full command packet. """
        self.desired_rudder_angle = msg.data
        self._send_command_to_teensy()

    def jib_angle_callback(self, msg):
        """ Handle ``jib_angle`` updates and forward the full command packet. """
        self.desired_jib_angle = msg.data
        self._send_command_to_teensy()

    def jib_side_flag_callback(self, msg):
        """ Handle ``jib_side_flag`` updates and forward the full command packet. """
        self.desired_jib_side_flag = int(msg.data)
        self._send_command_to_teensy()


def main(args=None):
    rclpy.init(args=args)
    teensy_node = Teensy()

    try:
        rclpy.spin(teensy_node)
    except KeyboardInterrupt:
        pass

    teensy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
