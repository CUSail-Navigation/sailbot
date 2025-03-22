import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import threading
from sailboat_interface.srv import Waypoint


class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')

        # Use for lock thread safety given that we read and write from our mutable self.waypoints field
        self.lock = threading.Lock()

        # Create service to get, set, or pop waypoints
        self.service = self.create_service(Waypoint, 'mutate_waypoint_queue', self.waypoint_service_callback)

        # Get initial waypoints from config
        self.declare_parameter('waypoints', ["42.876003, -77.009598"])  # Dummy initial
        waypoints_param = self.get_parameter('waypoints').get_parameter_value().string_array_value

        self.waypoints = self.parse_waypoints_param(waypoints_param)
        self.current_index = 1  # Start at one because of dummy initial (0, 0)

        self.get_logger().info('Navigate service started')

    def parse_waypoints_param(self, waypoints_param):
        waypoints = []
        self.get_logger().info(f'Waypoints: {waypoints_param}')
        for s in waypoints_param:
            x, y = map(float, s.split(','))
            waypoints.append((x, y))
        return waypoints

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
                except Exception as e:
                    response.message = f"Failed to update waypoints: {str(e)}"
                    response.success = False
                    self.get_logger().error(f"Error parsing new waypoints: {str(e)}")

            elif request.command == "pop":
                if self.waypoints:
                    popped_waypoint = self.waypoints.pop()
                    response.message = f"Waypoint {popped_waypoint} removed successfully."
                    response.success = True
                    self.get_logger().info(f"Popped waypoint: {popped_waypoint}")
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
