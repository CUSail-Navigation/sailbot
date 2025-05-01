import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Bool

class TrimSail(Node):
    """
    The sailing algorithm responsible for changing the sail angle based on the 
    wind direction.
    """

    def __init__(self):
        super().__init__('trim_sail')
        
        #Subscription for wind direction
        self.wind_subscription = self.create_subscription(
            Int32,
            'wind',
            self.wind_callback,
            10)   

        self.danger_zone_subscription = self.create_subscription(
            Bool,
            'danger_zone',
            self.danger_zone_callback,
            10)
        self.in_danger_zone = False 
        # Publisher for sail angle
        self.publisher_sail = self.create_publisher(Int32, 'algo_sail', 10)
        

    def wind_callback(self, msg):
        """
        Use the wind data from msg to calculate the sail angle and publish the absolute value 
        of the sail angle
        """
        sail_angle = abs(self.setSail(msg.data))

        # Create an Int32 message and publish the sail angle
        msg = Int32()
        msg.data = sail_angle

        self.publisher_sail.publish(msg)
        self.get_logger().info('Sail Angle: "%s"' % sail_angle)

    def danger_zone_callback(self, msg):
        """
        Callback function to handle the danger zone status.
        """
        self.in_danger_zone = msg.data

    def setSail(self, windDir):
        """
        Calculate and set the new sail angle given wind direction.
        Parameter precondition: windDir, an int in [0,360)
        Return value: the sail angle, an int in [-90,90]
        """
        # wind is blowing in the same direction as the sailing direction (run), this range
        # sets a 20 degree buffer zone so that the sail does not always flip.

        if 0 <= windDir < 10 or 350 < windDir < 360:
            return 90
        elif 235 < windDir <= 350: # assumes no-go zone is 55 degrees
            return round(((18/23)*windDir - 4230/23))
        elif 10 <= windDir < 125:
            return round((-(18/23)*windDir + 2250/23))
        # no go zone 
        else:
            return 0


def main(args=None):
    rclpy.init(args=args)

    trim_sail = TrimSail()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    try:
        rclpy.spin(trim_sail)
    except KeyboardInterrupt:
        pass
    
    trim_sail.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()