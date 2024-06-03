import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from sailboat_sensors.sailboat_sensors import SailAirMar


class ReadAirmar(Node):
    def __init__(self):
        super().__init__('ReadAirmar')

        # Declare timer period parameter
        self.declare_parameter('timer_period', 1.0)
        self.timer_period = self.get_parameter('timer_period').value

        self.publisher_ = self.create_publisher(Pose2D, 'position', 10)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.airmar = SailAirMar()

    def getHeading(self):
        return self.airmar.readAirMarHeading()
    def getX(self):
        return self.airmar.readAirMarLatitude()
    def getY(self):
        return self.airmar.readAirMarLongitude()

    def timer_callback(self):
        msg = Pose2D()
        msg.x = self.getX()
        msg.y = self.getY()
        msg.theta = self.getHeading()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Airmar Data: ' + 'x: ' + str(msg.x) + ' y: ' + str(msg.y) + ' theta: ' + str(msg.theta))


def main(args=None):
    rclpy.init(args=args)

    read_airmar = ReadAirmar()

    rclpy.spin(read_airmar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_airmar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

