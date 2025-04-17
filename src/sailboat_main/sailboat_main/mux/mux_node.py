import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class MuxNode(Node):
    def __init__(self):
        super().__init__('mux_node')

        # Default control mode
        self.declare_parameter('control_mode', 'algorithm')
        self.control_mode = self.get_parameter('control_mode').value

        # Mux topic for changing control_mode
        self.create_subscription(String, 'control_mode', self.control_mode_callback, 10)

        # Define control sources
        self.control_sources = {
            'radio': {'sail': None, 'rudder': None},
            'algorithm': {'sail': None, 'rudder': None},
            'webserver': {'sail': None, 'rudder': None},
            'controller_app': {'sail': None, 'rudder': None}
        }

        # Create subscribers for all control sources
        for source in self.control_sources:
            self.create_subscription(
                Int32, f'{source}_sail', 
                lambda msg, src=source: self.sail_callback(msg, src), 10)
            self.create_subscription(
                Int32, f'{source}_rudder', 
                lambda msg, src=source: self.rudder_callback(msg, src), 10)

        # Publishers for multiplexed output
        self.rudder_pub = self.create_publisher(Int32, 'rudder_angle', 10)
        self.sail_pub = self.create_publisher(Int32, 'sail', 10)

        # Timer to publish at a regular interval (10 Hz)
        self.create_timer(0.1, self.publish_muxed_values)

    def sail_callback(self, msg, source):
        self.control_sources[source]['sail'] = msg.data
        self.get_logger().info(f'{source} sail angle: {msg.data}')

    def rudder_callback(self, msg, source):
        self.control_sources[source]['rudder'] = msg.data
        self.get_logger().info(f'{source} rudder angle: {msg.data}')

    def control_mode_callback(self, msg):
        if msg.data in self.control_sources:
            self.control_mode = msg.data
            self.get_logger().info(f'Switched control mode to {msg.data}')
        else:
            self.get_logger().info(f'Invalid control mode request: {msg.data}. Staying in current mode ({self.control_mode})')

    def publish_muxed_values(self):
        # Get current values from the active control source
        sail_value = self.control_sources[self.control_mode]['sail']
        rudder_value = self.control_sources[self.control_mode]['rudder']
        
        # Publish sail value if available
        if sail_value is not None:
            sail_msg = Int32()
            sail_msg.data = sail_value
            self.sail_pub.publish(sail_msg)
            self.get_logger().info(f'Published sail angle from {self.control_mode}: {sail_value}')
        
        # Publish rudder value if available
        if rudder_value is not None:
            rudder_msg = Int32()
            rudder_msg.data = rudder_value
            self.rudder_pub.publish(rudder_msg)
            self.get_logger().info(f'Published rudder angle from {self.control_mode}: {rudder_value}')


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