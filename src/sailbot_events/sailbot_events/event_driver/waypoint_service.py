import rclpy
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import NavSatFix
# import pyproj
from sailboat_interface.srv import Waypoint

class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')
        self.service = self.create_service(Waypoint, 'get_waypoint', self.provide_waypoint_callback)
        default_points = [
        "42.876400267,-77.00778818759738",
        "42.876373,-77.007596",
        "42.876278,-77.007588"
        ]

        self.declare_parameter('waypoints', default_points)
        waypoints_param = self.get_parameter('waypoints').get_parameter_value().string_array_value #TODO: Change to FIFO queue

        self.waypoints = self.parse_waypoints(waypoints_param)
        self.current_index = 0

        self.get_logger().info('Navigate service started')

    def parse_waypoints(self, waypoints_param):
        waypoints = []
        self.get_logger().info(f'Waypoints: {waypoints_param}')
        for s in waypoints_param:
            x,y = map(float, s.split(','))
            waypoints.append([x,y])
        return waypoints


    def provide_waypoint_callback(self, request, response):
        self.get_logger().info('Incoming Request')
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            response.waypoint = NavSatFix()
            response.waypoint.header.frame_id = 'map'
            response.waypoint.header.stamp = self.get_clock().now().to_msg()
            response.waypoint.longitude = float(waypoint[0])
            response.waypoint.latitude = float(waypoint[1])
            response.success = True
            self.current_index += 1
        else:
            self.get_logger().info('No waypoints in list')

            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()