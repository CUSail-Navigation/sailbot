import rclpy
from rclpy.node import Node
from sailboat_interface.msg import SailTail
from std_msgs.msg import Int32

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
        self.declare_parameter('sail_port', '/dev/serial/by-id/usb-Teensyduino_Triple_Serial_16089010-if02')
        self.declare_parameter('tail_port', '/dev/serial/by-id/usb-Teensyduino_Triple_Serial_16089010-if04') #TODO: Change to by-id
        self.declare_parameter('simulated', False)

        self.sail_port = self.get_parameter('sail_port').value
        self.tail_port = self.get_parameter('tail_port').value


        self.simulated = self.get_parameter('simulated').value

        # Initialize the appropriate servo handler based on the simulation mode
        if self.simulated: #TODO: Fix sim mode for new serial setup
            self.sail = sail_servo_fake.SailServoFake()
            self.tail = sail_servo_fake.SailServoFake()
            self.get_logger().info('Simulation mode enabled. Serial communication is disabled.')
        else:
            self.sail = sail_servo.SailServo(self.sail_port)
            self.tail = sail_servo.SailServo(self.tail_port)
            self.get_logger().info(f'Real mode enabled. Serial communication on {self.sail_port} and {self.tail_port} .')

        self.subscription = self.create_subscription(
            Int32,
            'rudder_angle',
            self.rudder_callback,
            10)
        
        self.subscription = self.create_subscription(
            Int32,
            'sail',
            self.sail_callback,
            10)

        # Subscription to SailTail messages
        # self.subscription = self.create_subscription(
        #     SailTail,
        #     'servo',
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

    # def listener_callback(self, msg):
    #     """
    #     Callback function for the 'servo' topic. Delegates the message to the servo handler.
    #     """
    #     self.servo_handler.send_command(msg.sail, msg.tail)
    #     self.get_logger().info(f'Received Sail: {msg.sail}, Tail: {msg.tail}')

    
    def sail_callback(self, msg):
        """
        Callback function for the 'sail' topic. Delegates the message to the sail handler.
        """
        self.sail.send_command(msg.data, 0)
      #  self.get_logger().info(f'Received Sail: {msg.data}')
    
    def rudder_callback(self, msg):
        """
        Callback function for the 'rudder_angle' topic. Delegates the message to the rudder handler.
        """
        self.tail.send_command(msg.data, 0)
      #  self.get_logger().info(f'Received Rudder Angle: {msg.data}')

    def destroy_node(self):
        """
        Override the destroy_node method to clean up resources used by the servo handler.
        """
        self.sail.close()
        self.tail.close()

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
