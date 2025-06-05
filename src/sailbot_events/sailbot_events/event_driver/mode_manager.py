import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from sailboat_interface.srv import SetModeWithParams
from sailboat_interface.msg import StationRectangle

ALLOWED_MODES = {"manual", "station_keeping", "search"}

class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager')

        # Current mode and its parameters
        self.current_mode = "manual"
        self.station_rect = []  # List of 4 geometry_msgs/Point for rectangle corners
        self.sail_points = []  # List of 2 geometry_msgs/Point for sailing between
        self.search_center = None  # geometry_msgs/Point

        # Publisher for mode name
        self.mode_pub = self.create_publisher(String, 'current_mode', 10)
        # Publisher for station rectangle message
        self.station_rect_pub = self.create_publisher(StationRectangle, 'station_rectangle', 10)
        # Publisher for search center point
        self.search_center_pub = self.create_publisher(Point, 'search_center_point', 10)

        # Service to set mode and parameters
        self.set_mode_srv = self.create_service(SetModeWithParams, 'set_mode', self.set_mode_callback)

        # Publish initial mode
        self.publish_current_mode()
        self.get_logger().info(f"Initialized in mode '{self.current_mode}'")

    def publish_current_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def publish_station_rect(self):
        msg = StationRectangle()
        msg.corners = self.station_rect
        msg.sail_points = self.sail_points
        self.station_rect_pub.publish(msg)

    def publish_search_center(self):
        if self.search_center:
            self.search_center_pub.publish(self.search_center)

    def set_mode_callback(self, request, response):
        mode = request.mode.lower()

        if mode not in ALLOWED_MODES:
            response.success = False
            response.message = f"Invalid mode '{mode}'. Allowed: {list(ALLOWED_MODES)}"
            self.get_logger().warn(response.message)
            return response

        # If switching to same mode, still publish parameters
        self.current_mode = mode

        if mode == "station_keeping":
            # Expect exactly 4 points for rectangle and 2 points for sailing
            if len(request.station_rect_points) != 6:
                response.success = False
                response.message = "Station keeping mode requires exactly 6 points (4 for rectangle, 2 for sailing)."
                return response
            self.station_rect = request.station_rect_points[:4]
            self.sail_points = request.station_rect_points[4:]
            self.get_logger().info(f"[ModeManager] Station keeping with 4 rectangle points and 2 sail points.")

        elif mode == "search":
            self.search_center = request.search_center_point
            self.get_logger().info(f"[ModeManager] Search mode with center point ({self.search_center.y}, {self.search_center.x})")

        elif mode == "manual":
            # Clear parameters
            self.station_rect = []
            self.sail_points = []
            self.search_center = None
            self.get_logger().info("[ModeManager] Manual mode activated.")

        # Publish new mode and parameters
        self.publish_current_mode()
        if mode == "station_keeping":
            self.publish_station_rect()
        elif mode == "search":
            self.publish_search_center()

        response.success = True
        response.message = f"Switched to '{mode}' mode and parameters published."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
