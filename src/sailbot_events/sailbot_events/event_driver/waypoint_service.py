import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import threading
from sailboat_interface.srv import Waypoint
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')

        # Use for lock thread safety given that we read and write from our mutable self.waypoints field
        self.lock = threading.Lock()

        # Create service to get, set, or pop waypoints
        self.service = self.create_service(Waypoint, 'mutate_waypoint_queue', self.waypoint_service_callback)
        
        # Create publisher for current waypoint
        self.waypoint_publisher = self.create_publisher(NavSatFix, 'current_waypoint', 10)

        # Get initial waypoints from config
        self.declare_parameter('waypoints', [""], ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description="List of waypoints in the format 'lat,lon' separated by ',. Example: '37.7749,-122.4194,37.7749,-122.4195'",
        ))  # Default to an empty list if no waypoints are provided
        waypoints_param = self.get_parameter('waypoints').get_parameter_value().string_array_value

        # this is a workaround for if no defaults are set; ROS requires a parameter string array be non-empty when declaring a parameter
        # but we want a default truly empty waypoint queue
        self.waypoints = self.parse_waypoints_param(waypoints_param) if waypoints_param != [''] else []

        self.publish_current_waypoint() # Publish the first waypoint if self.waypoints is non-empty

        self.get_logger().info('Navigate service started')

    def parse_waypoints_param(self, waypoints_param):
        waypoints = []
        self.get_logger().info(f'Waypoints: {waypoints_param}')
        for s in waypoints_param:
            x, y = map(float, s.split(','))
            waypoints.append((x, y))
        return waypoints
    
    def publish_current_waypoint(self):
        """
        Publish the current waypoint (front of queue) as a NavSatFix message.
        Assumes coordinates are in latitude, longitude format.
        """
        if self.waypoints == []:
            self.get_logger().warning("No waypoints to publish")
            return
            
        # Create NavSatFix message
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Assuming the waypoint coordinates are (latitude, longitude)
        msg.latitude = self.waypoints[0][0]
        msg.longitude = self.waypoints[0][1]
        
        # Set other NavSatFix fields
        msg.altitude = 0.0  # Set to 0 or appropriate value
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        
        # Publish the message
        self.waypoint_publisher.publish(msg)
        self.get_logger().info(f"Published current waypoint: Lat={msg.latitude}, Lon={msg.longitude}")

    def waypoint_service_callback(self, request, response):
        with self.lock:
            self.get_logger().info(f'Received request: command={request.command}, argument={request.argument}')

            if request.command == "get":
                # Return string representation of the waypoint queue
                response.message = str(self.waypoints)
                response.success = True

            elif request.command == "set":
                try:
                    # Parse the string argument into a new waypoint queue
                    waypoints = []
                    for pair in request.argument.split(';'):  # Waypoints separated by ';'
                        x, y = map(float, pair.split(','))
                        waypoints.append((x, y))

                    # Replace the entire waypoint queue
                    self.waypoints = waypoints
                    response.message = "Waypoint queue successfully updated"
                    response.success = True
                    self.get_logger().info(f"New waypoint queue: {self.waypoints}")
                    
                    # Publish the current (first) waypoint if available
                    if self.waypoints:
                        self.publish_current_waypoint()
                except Exception as e:
                    response.message = f"Failed to update waypoints: {str(e)}"
                    response.success = False
                    self.get_logger().error(f"Error parsing new waypoints: {str(e)}")

            elif request.command == "pop":
                if self.waypoints:
                    popped_waypoint = self.waypoints.pop(0)  # Pop from front of queue
                    response.message = f"Waypoint {popped_waypoint} removed successfully."
                    response.success = True
                    self.get_logger().info(f"Popped waypoint: {popped_waypoint}")
                    
                    # Publish the new current waypoint if available
                    if self.waypoints:
                        self.publish_current_waypoint()
                else:
                    response.message = "No waypoints to pop. The queue is empty."
                    response.success = False
                    self.get_logger().warning("Attempted to pop from an empty waypoint queue.")

            else:
                response.message = "Invalid command. Use 'get', 'set', or 'pop'."
                response.success = False
                self.get_logger().error("Invalid command received")

            return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# Command to call the 'pop' command on the 'mutate_waypoint_queue' service
# Run this in the terminal
# ros2 service call /mutate_waypoint_queue sailboat_interface/srv/Waypoint "{command: 'pop', argument: ''}"