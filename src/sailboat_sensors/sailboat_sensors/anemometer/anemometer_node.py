import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from . import sail_anemometer
from . import sail_anemometer_fake

class Anemometer(Node):
    """
    This class is a ROS2 node that publishes wind data from the anemometer sensor.
    It uses the SailAnemometer class for sensor communications.
    """
    def __init__(self):
        super().__init__('anemometer')

        # Declare parameters
        self.declare_parameter('timer_period', 0.5)
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('anemometer_port', 'usb-Teensyduino_Triple_Serial_16089010-if00') # FIXME: Add a better default port
        anemometer_port = self.get_parameter('anemometer_port').value

        self.declare_parameter('use_fake_data', False)
        self.use_fake_data = self.get_parameter('use_fake_data').value

        # Create a SailAnemometer instance for sensor communications
        if self.use_fake_data:
            self.sail_anemometer = sail_anemometer_fake.FakeSailAnemometer()
        else:
            self.sail_anemometer = sail_anemometer.SailAnemometer(port=anemometer_port)

        # Create a ROS publisher
        self.publisher_ = self.create_publisher(Int32, '/wind', 10)

        # Create a timer to periodically read the wind data and publish it
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('Launching Anemometer')

    def timer_callback(self):
        """
        Callback function for the timer. It reads the wind data from the sensor
        and publishes it to the '/wind' topic.
        """
        msg = Int32()
        wind_data = self.sail_anemometer.read_wind()
        if wind_data is not None:
            msg.data = wind_data
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing Wind Angle: {msg.data}')

    def destroy_node(self):
        """
        Override the destroy_node method to close the serial connection before shutting down.
        """
        self.sail_anemometer.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    anemometer_node = Anemometer()

    try:
        rclpy.spin(anemometer_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    anemometer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()