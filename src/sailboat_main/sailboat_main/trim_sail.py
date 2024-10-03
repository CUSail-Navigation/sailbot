import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class TrimSail(Node):

    def __init__(self):
        super().__init__('trim_sail')
        # self.subscription = self.create_subscription(
        #     Imu,
        #     '/imu',
        #     self.imu_callback,
        #     10)
        # self.subscription  # prevent unused variable warning
        
        self.subscription = self.create_subscription(
            Int32,
            '/wind',
            self.wind_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Int32, '/sail', 10)

    def wind_callback(self, msg):
        sail_angle = abs(self.setSail(msg.data))
        self.publisher_.publish(sail_angle)
        self.get_logger().info('Sail Angle: "%s"' % sail_angle)

    def setSail(self, windDir):
        """
        Calculate and set the new sail angle given ... Figure out what parameters are needed
        """
        # wind is blowing in the same direction as the sailing direction (run), this range
        # sets a 20 degree buffer zone so that the sail does not always flip.
        if 0 <= windDir < 10 or 350 < windDir < 360:
            return 90
        elif 210 < windDir <= 350:
            return round(((7/15)*windDir - 80)/5)*5
        elif 10 <= windDir < 150:
            return round(((7/15)*windDir - 88)/5)*5
        # no go zone (150 <= cWindDir <= 210)
        else:
            return 0


def main(args=None):
    rclpy.init(args=args)

    trim_sail = TrimSail()

    rclpy.spin(trim_sail)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trim_sail.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()