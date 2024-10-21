import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import random
import time


class EventDriverNode(Node):
    def __init__(self):
        super().__init__('event_driver_node')
        self.publisher_ = self.create_publisher(NavSatFix, '/current_destination', 10)
        self.timer = self.create_timer(1.0, self.publish_destination)  # Publish every 2 seconds
        self.get_logger().info('Event Driver Node has started.')
 
    def publish_destination(self):
        """
        Publishes a random GPS location to the '/current_destination' topic.
        """
        # Create a NavSatFix message
        gps_msg = NavSatFix()

        # Set latitude range to avoid exceeding UTM bounds (valid latitude is between -80 and 84)
        gps_msg.latitude = random.uniform(-80.0, 84.0)  # Random latitude within UTM valid range
        gps_msg.longitude = random.uniform(-180.0, 180.0)  # Random longitude

        # Log the GPS message being published
        self.get_logger().info(f'Publishing destination: latitude {gps_msg.latitude}, '
                               f'longitude {gps_msg.longitude}, altitude {gps_msg.altitude}')
        
        # Publish the message
        self.publisher_.publish(gps_msg)


def main(args=None):
    rclpy.init(args=args)
    event_driver_node = EventDriverNode()

    try:
        rclpy.spin(event_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        event_driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
