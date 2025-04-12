import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String # Assuming the sail and rudder topics use Int32 messages

class MuxNode(Node):
    def __init__(self):
        super().__init__('mux_node')

        self.declare_parameter('control_node', 'algorithm')
        self.control_mode = self.get_parameter('control_mode').value

        # Mux topic for changing control_mode topic
        self.control_mode_sub = self.create_subscription(
            String, 'control_mode', self.control_mode_callback, 10)

        # Subscribers for radio topics
        self.radio_sail_sub = self.create_subscription(
            Int32, 'radio_sail', self.radio_sail_callback, 10)
        self.radio_rudder_sub = self.create_subscription(
            Int32, 'radio_rudder', self.radio_rudder_callback, 10)

        # Subscribers for algorithm topics
        self.algo_sail_sub = self.create_subscription(
            Int32, 'algo_sail', self.algo_sail_callback, 10)
        self.algo_rudder_sub = self.create_subscription(
            Int32, 'algo_rudder', self.algo_rudder_callback, 10)
        
        # Subscribers for webserver topics
        self.algo_sail_sub = self.create_subscription(
            Int32, 'webserver_sail', self.algo_sail_callback, 10)
        self.algo_rudder_sub = self.create_subscription(
            Int32, 'webserver_rudder', self.algo_rudder_callback, 10)

        # Publishers for multiplexed output (rudder_angle and sail)
        self.rudder_pub = self.create_publisher(Int32, 'rudder_angle', 10)
        self.sail_pub = self.create_publisher(Int32, 'sail', 10)

        # Store the latest values from the topics
        self.radio_sail = None
        self.radio_rudder = None
        self.algo_sail = None
        self.algo_rudder = None
        self.webserver_sail = None
        self.webserver_rudder = None

        # Timer to publish at a regular interval (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.publish_muxed_values)

    def radio_sail_callback(self, msg):
        self.radio_sail = msg.data

    def radio_rudder_callback(self, msg):
        self.radio_rudder = msg.data

    def algo_sail_callback(self, msg):
        self.algo_sail = msg.data

    def algo_rudder_callback(self, msg):
        self.algo_rudder = msg.data

    def webserver_sail_callback(self, msg):
        self.webserver_sail = msg.data

    def webserver_rudder_callback(self, msg):
        self.webserver_rudder = msg.data

    def control_mode_callback(self, msg):
        if msg.data == 'radio' or msg.data == 'algorithm' or msg.data == 'webserver': 
            self.control_mode = msg.data
            self.get_logger().info(f'Switched control mode to {msg.data}')
        else:
            self.get_logger().info(f'Invalid control mode request: {msg.data}. Staying in current mode ({self.control_mode})')

    def publish_muxed_values(self):
        # Mux logic: switch between radio or algorithm input based on `use_radio`
        if self.control_mode == 'radio':
            # Use radio control
            if self.radio_sail is not None:
                sail_msg = Int32()
                sail_msg.data = self.radio_sail
                self.sail_pub.publish(sail_msg)
                self.get_logger().info(f'Published sail angle from radio: {self.radio_sail}')

            if self.radio_rudder is not None:
                rudder_msg = Int32()
                rudder_msg.data = self.radio_rudder
                self.rudder_pub.publish(rudder_msg)
                self.get_logger().info(f'Published rudder angle from radio: {self.radio_rudder}')

        elif self.control_mode == 'algorithm':
            # Use algorithm control
            if self.algo_sail is not None:
                sail_msg = Int32()
                sail_msg.data = self.algo_sail
                self.sail_pub.publish(sail_msg)
                self.get_logger().info(f'Published sail angle from algorithm: {self.algo_sail}')

            if self.algo_rudder is not None:
                rudder_msg = Int32()
                rudder_msg.data = self.algo_rudder
                self.rudder_pub.publish(rudder_msg)
                self.get_logger().info(f'Published rudder angle from algorithm: {self.algo_rudder}')

        elif self.control_mode == 'webserver':
            # Use algorithm control
            if self.webserver_sail is not None:
                sail_msg = Int32()
                sail_msg.data = self.webserver_sail
                self.sail_pub.publish(sail_msg)
                self.get_logger().info(f'Published sail angle from webserver: {self.webserver_sail}')

            if self.webserver_rudder is not None:
                rudder_msg = Int32()
                rudder_msg.data = self.webserver_rudder
                self.rudder_pub.publish(rudder_msg)
                self.get_logger().info(f'Published rudder angle from webserver: {self.webserver_rudder}')


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
