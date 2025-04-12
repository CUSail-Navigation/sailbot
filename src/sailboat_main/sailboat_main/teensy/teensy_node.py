import rclpy
from rclpy.node import Node
from sailboat_interface.msg import SailTail
from std_msgs.msg import Int32
from std_msgs.msg import String

from . import teensy
from . import teensy_fake
 


class Teensy(Node):
    """
    ROS 2 Node that interfaces with a Teensy microcontroller to control and 
    monitor the Sailbot's hardware, including the sail and rudder. It 
    communicates with the Teensy either in real mode (via serial communication) 
    or simulated mode, depending on configuration parameters. The class handles 
    reading telemetry data (e.g., wind angle, sail angle, rudder angle, and 
    dropped packets) and sending control commands to the sail and rudder via 
    ROS topic subscriptions.
    """
    def __init__(self):
        super().__init__('teensy')

        # declare parameters 
        self.declare_parameter('teensy_port', '/dev/ttyACM0')
        self.declare_parameter('simulated', False)

        # teensy sends data every 0.5s, chose .25s read period to avoid overflow   
        self.declare_parameter('rx_period', 0.500) 

        # get parameters
        self.timer_period = self.get_parameter('rx_period').value
        self.teensy_port = self.get_parameter('teensy_port').value
        self.simulated = self.get_parameter('simulated').value

        # initialize the appropriate servo handler based on the simulation mode
        if self.simulated: 
            self.teensy = teensy_fake.TeensyFake()
            self.get_logger().info('Simulation mode enabled. Serial communication is disabled.')
        else:
            self.teensy = teensy.TeensyHardware(self.teensy_port)
            self.get_logger().info(f'Real mode enabled. Serial communication on {self.teensy_port}.')
        
        # keep track of our desired sail and rudder angles
        self.desired_sail = 0
        self.desired_rudder = 0

        # Control mode
        self.autonomous_mode = False  # Default to RC Mode

        # telemetry data publishers
        self.wind_angle_pub = self.create_publisher(Int32, 'wind', 10)
        self.actual_sail_angle_pub = self.create_publisher(Int32, 'actual_sail_angle', 10)
        self.actual_rudder_angle_pub = self.create_publisher(Int32, 'actual_rudder_angle', 10)
        self.dropped_packets_pub = self.create_publisher(Int32, 'dropped_packets', 10)

        # callback to read teensy data
        self.timer = self.create_timer(self.timer_period, self.check_telemetry)

        # subscriptions to sail and rudder topics
        self.subscription = self.create_subscription(
            Int32,
            'sail',
            self.sail_callback,
            10)

        self.subscription = self.create_subscription(
            Int32,
            'rudder_angle',
            self.rudder_callback,
            10)
        
        # subscription to control mode topic
        self.subscription = self.create_subscription(
            String,
            'control_mode',
            self.control_mode_callback,
            10)
    
    def check_telemetry(self):
        """
        Timer based function that checks for teensy telemetry data. Publishes
        data to telemetry topics.
        """
        self.get_logger().info("Check telemetry callback entered")
        data = {}
        if (self.teensy.read_telemetry(data) == 0):
            wind_angle_msg = Int32()
            wind_angle_msg.data = data["wind_angle"]
            self.wind_angle_pub.publish(wind_angle_msg)

            sail_angle_msg = Int32()
            sail_angle_msg.data = data["sail_angle"]
            self.actual_sail_angle_pub.publish(sail_angle_msg)

            rudder_angle_msg = Int32()
            rudder_angle_msg.data = data["rudder_angle"]
            self.actual_rudder_angle_pub.publish(rudder_angle_msg)

            dropped_packets_msg = Int32()
            dropped_packets_msg.data = data["dropped_packets"]
            self.dropped_packets_pub.publish(dropped_packets_msg)

            self.get_logger().info(f"{'Wind angle:':<20} {wind_angle_msg.data}")
            self.get_logger().info(f"{'Actual sail angle:':<20} {sail_angle_msg.data}")
            self.get_logger().info(f"{'Actual tail angle:':<20} {rudder_angle_msg.data}")
            self.get_logger().info(f"{'Dropped packets:':<20} {dropped_packets_msg.data}")
        else:
            self.get_logger().info("No telemetry received")

    def sail_callback(self, msg):
        """
        Callback function for the 'sail' topic. Sends the updated sail and the
        previously set rudder.
        """
        self.desired_sail = msg.data
        #self.get_logger().info(f"Sail callback-sent to Teensy sail:{self.desired_sail}, rudder: {self.desired_rudder}")

        if self.teensy.send_command(self.desired_sail, self.desired_rudder, self.autonomous_mode) == 0:
            self.get_logger().info(f"Message sent to servo")
        else:
            self.get_logger().warn(f"Message failed to send to servo")
    

    def rudder_callback(self, msg):
        """
        Callback function for the 'rudder_angle' topic. Sends the updated rudder
        and the previously set sail.
        """
        self.desired_rudder = msg.data
        self.get_logger().info(f"Rudder callback-sent to Teensy sail:{self.desired_sail}, rudder: {self.desired_rudder}")

        if self.teensy.send_command(self.desired_sail, self.desired_rudder, self.autonomous_mode) == 0:
            self.get_logger().info(f"Message sent to servo")
        else:
            self.get_logger().warn(f"Message failed to send to servo")

    def control_mode_callback(self, msg):
        if msg.data == 'radio':
            self.autonomous_mode = False  # RC Mode
        elif msg.data == 'algorithm':
            self.autonomous_mode = True  # Autonomous Mode
        self.get_logger().info(f"Control Mode Updated: {msg.data}")

    def destroy_node(self):
        """
        Override the destroy_node method to clean up resources used by the handler.
        """
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    teensy = Teensy()

    try:
        rclpy.spin(teensy)
    except KeyboardInterrupt:
        pass

    teensy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
