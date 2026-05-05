import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, UInt8

class MuxNode(Node):
    def __init__(self):
        super().__init__('mux_node')

        # Default control mode.
        self.declare_parameter('control_mode', 'algo')
        self.control_mode = self.get_parameter('control_mode').value

        # Mux topic for changing control_mode.
        self.create_subscription(String, 'control_mode', self.control_mode_callback, 10)

        # Define control sources.
        self.control_sources = {
            'radio': {'sail': None, 'rudder': None, 'jib_angle': None, 'jib_side_flag': None},
            'algo': {'sail': None, 'rudder': None, 'jib_angle': None, 'jib_side_flag': None},
            'webserver': {'sail': None, 'rudder': None, 'jib_angle': None, 'jib_side_flag': None},
            'controller_app': {'sail': None, 'rudder': None, 'jib_angle': None, 'jib_side_flag': None},
        }

        # Create subscribers for all control sources.
        for source in self.control_sources:
            self.create_subscription(
                Int32, f'{source}_sail',
                lambda msg, src=source: self.mainsail_angle_callback(msg, src), 10)
            self.create_subscription(
                Int32, f'{source}_rudder',
                lambda msg, src=source: self.rudder_angle_callback(msg, src), 10)
            self.create_subscription(
                Int32, f'{source}_jib_angle',
                lambda msg, src=source: self.jib_angle_callback(msg, src), 10)
            self.create_subscription(
                UInt8, f'{source}_jib_side_flag',
                lambda msg, src=source: self.jib_side_flag_callback(msg, src), 10)

        # Publishers for multiplexed output.
        self.mainsail_angle_pub = self.create_publisher(Int32, 'sail', 10)
        self.rudder_angle_pub = self.create_publisher(Int32, 'rudder_angle', 10)
        self.jib_angle_pub = self.create_publisher(Int32, 'jib_angle', 10)
        self.jib_side_flag_pub = self.create_publisher(UInt8, 'jib_side_flag', 10)

        # Timer to publish at a regular interval (10 Hz).
        self.create_timer(0.1, self.publish_muxed_values)

    def mainsail_angle_callback(self, msg, source):
        self.control_sources[source]['sail'] = msg.data
        self.get_logger().info(f'{source} mainsail angle: {msg.data}')

    def rudder_angle_callback(self, msg, source):
        self.control_sources[source]['rudder'] = msg.data
        self.get_logger().info(f'{source} rudder angle: {msg.data}')

    def jib_angle_callback(self, msg, source):
        self.control_sources[source]['jib_angle'] = msg.data
        self.get_logger().info(f'{source} jib angle: {msg.data}')

    def jib_side_flag_callback(self, msg, source):
        self.control_sources[source]['jib_side_flag'] = msg.data
        self.get_logger().info(f'{source} jib side flag: {msg.data}')

    def control_mode_callback(self, msg):
        if msg.data in self.control_sources:
            self.control_mode = msg.data
            self.get_logger().info(f'Switched control mode to {msg.data}')
        else:
            self.get_logger().info(
                f'Invalid control mode request: {msg.data}. '
                f'Staying in current mode ({self.control_mode}).'
            )

    def publish_muxed_values(self):
        # Get current values from the active control source.
        src = self.control_sources[self.control_mode]
        mainsail_value = src['sail']
        rudder_value = src['rudder']
        jib_angle_value = src['jib_angle']
        jib_side_value = src['jib_side_flag']

        # Publish mainsail value if available.
        if mainsail_value is not None:
            mainsail_msg = Int32()
            mainsail_msg.data = mainsail_value
            self.mainsail_angle_pub.publish(mainsail_msg)
            self.get_logger().info(f'Published mainsail angle from {self.control_mode}: {mainsail_value}')

        # Publish rudder value if available.
        if rudder_value is not None:
            rudder_msg = Int32()
            rudder_msg.data = rudder_value
            self.rudder_angle_pub.publish(rudder_msg)
            self.get_logger().info(f'Published rudder angle from {self.control_mode}: {rudder_value}')

        # Publish jib values only when both are set, so downstream never gets a half-update from mux.
        if jib_angle_value is not None and jib_side_value is not None:
            jib_msg = Int32()
            jib_msg.data = jib_angle_value
            jib_side_msg = UInt8()
            jib_side_msg.data = jib_side_value
            self.jib_angle_pub.publish(jib_msg)
            self.jib_side_flag_pub.publish(jib_side_msg)
            self.get_logger().info(
                f'Published jib angle and side from {self.control_mode}: '
                f'angle={jib_angle_value}, side={jib_side_value}'
            )


def main(args=None):
    rclpy.init(args=args)
    mux_node = MuxNode()

    try:
        rclpy.spin(mux_node)
    except KeyboardInterrupt:
        pass

    mux_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
