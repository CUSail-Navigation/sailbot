import rclpy
from rclpy.node import Node
from sailboat_interface.msg import SailTail
from . import sail_servo  # Import the real SailServo class
from . import sail_servo_fake  # Import the simulated SailServo class

class Servo(Node):
    """
    ROS2 Node responsible for subscribing to the 'servo' topic and
    delegating the received messages to the appropriate servo handler.
    """
    def __init__(self):
        super().__init__('servo')

        # Declare parameters
        self.declare_parameter('servo_port', '/dev/ttyUSB0')
        self.declare_parameter('simulated', False)

        self.servo_port = self.get_parameter('servo_port').value
        self.simulated = self.get_parameter('simulated').value

        # Initialize the appropriate servo handler based on the simulation mode
        if self.simulated:
            self.servo_handler = sail_servo_fake.SailServoFake()
            self.get_logger().info('Simulation mode enabled. Serial communication is disabled.')
        else:
            self.servo_handler = sail_servo.SailServo(self.servo_port)
            self.get_logger().info(f'Real mode enabled. Serial communication on {self.servo_port}.')

        # Subscription to SailTail messages
        self.subscription = self.create_subscription(
            SailTail,
            'servo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        Callback function for the 'servo' topic. Delegates the message to the servo handler.
        """
        self.servo_handler.send_command(msg.sail, msg.tail)
        self.get_logger().info(f'Received Sail: {msg.sail}, Tail: {msg.tail}')

    def destroy_node(self):
        """
        Override the destroy_node method to clean up resources used by the servo handler.
        """
        self.servo_handler.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    servo = Servo()

    try:
        rclpy.spin(servo)
    except KeyboardInterrupt:
        pass

    servo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
