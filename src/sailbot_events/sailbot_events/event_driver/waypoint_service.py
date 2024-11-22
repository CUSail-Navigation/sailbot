import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import threading 
from sailboat_interface.srv import Waypoint

class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')

        # use for lock thread safety given that we read and write from our mutable self.waypoints field
        self.lock = threading.Lock()

        # create service to get waypoints
        self.service = self.create_service(Waypoint, 'get_waypoint', self.provide_waypoint_callback)

        # create subscription to receive new waypoints for queue
        self.create_subscription(String, 'waypoint_queue', self.waypoint_queue_callback, 10)

        # get initial waypoints from config
        self.declare_parameter('waypoints', ["0, 0"]) # dummy initial
        waypoints_param = self.get_parameter('waypoints').get_parameter_value().string_array_value

        self.waypoints = self.parse_waypoints_param(waypoints_param)
        self.current_index = 1 # start at one because of dummy initial (0, 0)

        self.get_logger().info('Navigate service started')

    def parse_waypoints_param(self, waypoints_param):
        waypoints = []
        self.get_logger().info(f'Waypoints: {waypoints_param}')
        for s in waypoints_param:
            x,y = map(float, s.split(','))
            waypoints.append((x,y))
        return waypoints
    
    def waypoint_queue_callback(self, msg):
        # parse msg of the form '{long}, {lat}'
        with self.lock:
            try: 
                lat, long = map(float, msg.data.split(', '))
                self.waypoints.append((lat, long))
                self.get_logger().info(f'Updated Waypoint queue: {self.waypoints}')
            except Exception as e:
                self.get_logger().info(f'Waypoint queue could not parse msg: {msg} due to error {str(e)}')

    def provide_waypoint_callback(self, request, response):
        with self.lock:
            self.get_logger().info('Incoming Request')
            if self.current_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_index]
                response.waypoint = NavSatFix()
                response.waypoint.header.frame_id = 'map'
                response.waypoint.header.stamp = self.get_clock().now().to_msg()
                response.waypoint.longitude = waypoint[0]
                response.waypoint.latitude = waypoint[1]
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