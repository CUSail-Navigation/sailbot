import numpy as np
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from rclpy.task import Future
from typing import Optional

from sailboat_interface.srv import Waypoint
from sailboat_interface.msg import AlgoDebug

import utm
import math

class UTMPoint():
    """
    A class to represent a point in UTM coordinates.
    """
    easting : float
    northing : float
    zone_number : int
    zone_letter : str

    def __init__(self, easting, northing, zone_number, zone_letter):
        self.easting = easting
        self.northing = northing
        self.zone_number = zone_number
        self.zone_letter = zone_letter

    def to_latlon(self) -> 'LatLongPoint':
        """
        Convert UTM coordinates to latitude and longitude.
        """
        latitude, longitude = utm.to_latlon(self.easting, self.northing, self.zone_number, self.zone_letter)
        return LatLongPoint(latitude, longitude) 

    def to_navsatfix_msg(self) -> NavSatFix:
        """
        Convert UTM coordinates to NavSatFix message.
        """
        lat_long = self.to_latlon()
        msg = NavSatFix()
        msg.latitude = lat_long.latitude
        msg.longitude = lat_long.longitude
        return msg
    
    def distance_to(self, other: 'UTMPoint') -> float:
        """
        Calculate the distance to another UTM point.
        """
        assert self.zone_number == other.zone_number, "Zone numbers must be the same for distance calculation"
        return math.dist((self.easting, self.northing), (other.easting, other.northing))

    def __repr__(self):
        return f"UTMPoint(x={self.x}, y={self.y}, zone_number={self.zone_number}, zone_letter={self.zone_letter})"

class LatLongPoint():
    """
    A class to represent a point in latitude and longitude coordinates.
    """
    latitude : float
    longitude : float
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def to_utm(self) -> 'UTMPoint':
        """
        Convert latitude and longitude to UTM coordinates.
        """
        x, y, zone_number, zone_letter = utm.from_latlon(self.latitude, self.longitude)
        return UTMPoint(x, y, zone_number, zone_letter) 

    def __repr__(self):
        return f"latitude={self.latitude}, longitude={self.longitude}"

class BuoySearch(Node):
    """
    The sailing algorithm responsible for search event.
    """
    wind_dir : Optional[float]
    curr_loc : Optional[UTMPoint]
    tacking : bool
    tacking_point : Optional[UTMPoint]
    tacking_buffer : int # cooldown between calculating tacking points
    no_go_zone : int # anglular size of the no-go-zone on one side of the boat's centerline
    heading_dir : Optional[float]
    curr_dest : Optional[UTMPoint]
    diff : Optional[float]
    dist_to_dest : Optional[float]

    def __init__(self):
        super().__init__('buoy_search')

        # self.declare_parameter('timer_period', 0.500) 
        # self.timer_period = self.get_parameter('timer_period').value

        # self.declare_parameter('tacking_buffer', 15)
        # self.tacking_buffer = self.get_parameter('tacking_buffer').value

        # self.declare_parameter("debug", False)
        # self.debug = self.get_parameter("debug").value
    
        # self.declare_parameter("no_go_zone", 45)
        # self.no_go_zone = self.get_parameter("no_go_zone").value

        # self.tack_time_tracker = 0

        #Subscription for current location
        self.subscription_curr_loc = self.create_subscription(
            NavSatFix,
            '/gps',
            self.curr_gps_callback,
            10)

        #Subscription for heading direction
        self.subscription_heading_dir = self.create_subscription(
            Vector3,
            '/imu',
            self.heading_dir_callback,
            10)

        #Subscription for wind direction
        self.subscription_wind_dir = self.create_subscription(
            Int32,
            'wind',
            self.wind_callback,
            10)

        # #Subscription for wind direction
        # self.subscription_wind_dir = self.create_subscription(
        #     NavSatFix,
        #     'current_waypoint',
        #     self.update_current_waypoint,
        #     10)

        # Subscription for buoy position
        self.subscription_buoy_pos = self.create_subscription(
            Point,
            '/buoy_position',
            self.buoy_position_callback,
            10)
        
        # self.timer = self.create_timer(self.timer_period, self.step)

        # # Publisher for rudder angle
        # self.rudder_angle_pub = self.create_publisher(Int32, 'algo_rudder', 10)

        # self.tacking_point_pub = self.create_publisher(NavSatFix, 'tacking_point', 10)

        # Internal state
        self.wind_dir = None
        self.curr_loc = None
        # self.tacking = False
        # self.tacking_point = None
        self.heading_dir = None
        # self.curr_dest = None
        # self.diff = None
        # self.dist_to_dest = None

        # if self.debug:
        #     # Publisher for internal state
        #     self.state_pub = self.create_publisher(AlgoDebug, 'main_algo_debug', 10)
        #     # Timer to publish state every 1 second
        #     self.state_timer = self.create_timer(1.0, self.publish_state_debug)

        # # Publisher for tacking point
        # self.tacking_point_pub = self.create_publisher(NavSatFix, 'tacking_point', 10)

        self.waypoints = []

        self.initialize_search_pattern()

        self.get_logger().info('Buoy search algo started successfully')  # Check if this line prints

    # def update_current_waypoint(self, msg):
    #     """
    #     Callback to update the current destination waypoint from the 'current_waypoint' topic.
    #     """
    #     self.curr_dest = LatLongPoint(msg.latitude, msg.longitude).to_utm()
    #     self.get_logger().info(f'Updated current waypoint to: ({msg.latitude}, {msg.longitude})')

    # def publish_state_debug(self):
    #     """
    #     Publish the internal state as a JSON string.
    #     """
    #     debug_msg = AlgoDebug()
    #     debug_msg.tacking = self.tacking

    #     if self.tacking_point is not None:
    #         debug_msg.tacking_point = self.tacking_point.to_navsatfix_msg()
    #     else:
    #         debug_msg.tacking_point = NavSatFix() # empty msg

    #     debug_msg.heading_dir = Int32()
    #     debug_msg.heading_dir.data = int(self.heading_dir) if self.heading_dir is not None else 0

    #     if self.curr_dest is not None:
    #         debug_msg.curr_dest = self.curr_dest.to_navsatfix_msg()
    #     else:
    #         debug_msg.curr_dest = NavSatFix() # empty msg

    #     debug_msg.diff = Int32()
    #     debug_msg.diff.data = int(self.diff) if self.diff is not None else 0

    #     if self.dist_to_dest is not None:
    #         debug_msg.dist_to_dest = Int32()
    #         debug_msg.dist_to_dest.data = int(self.dist_to_dest)
    #     else:
    #         debug_msg.dist_to_dest = Int32()
    #         debug_msg.dist_to_dest.data = -666

    #     self.state_pub.publish(debug_msg)


    def pop_waypoint(self):
        self.cli = self.create_client(Waypoint, 'mutate_waypoint_queue')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting...')

        # Set up the request with the "pop" command
        self.req = Waypoint.Request()
        self.req.command = "pop"
        self.req.argument = ""  # No argument needed for the "pop" command

        self.future = self.cli.call_async(self.req)
        # Use a callback to handle the response
        self.future.add_done_callback(self.pop_response_callback)

    def pop_response_callback(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Waypoint popped successfully.')
            else:
                self.get_logger().info('Failed to pop waypoint from the service.')
        except Exception as e:
            self.get_logger().error(f'Error in waypoint_response_callback: {str(e)}')
    
    def set_waypoints(self, waypoints):
        self.cli = self.create_client(Waypoint, 'mutate_waypoint_queue')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting...')

        self.waypoints = []
        self.waypoints.extend(waypoints)

        # Set up the request with the "set" command
        self.req = Waypoint.Request()
        self.req.command = "set"
        self.req.argument = ';'.join(self.waypoints)

        self.future = self.cli.call_async(self.req)
        # Use a callback to handle the response
        self.future.add_done_callback(self.push_response_callback)

    def set_response_callback(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Waypoint set successfully.')
            else:
                self.get_logger().info('Failed to set waypoint from the service.')
        except Exception as e:
            self.get_logger().error(f'Error in waypoint_response_callback: {str(e)}')

    def push_waypoint(self, waypoint):
        self.cli = self.create_client(Waypoint, 'mutate_waypoint_queue')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting...')

        self.waypoints.append(waypoint)

        # Set up the request with the "push" command
        self.req = Waypoint.Request()
        self.req.command = "push"
        self.req.argument = ""  # TODO

        self.future = self.cli.call_async(self.req)
        # Use a callback to handle the response
        self.future.add_done_callback(self.push_response_callback)

    def push_response_callback(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Waypoint pushed successfully.')
            else:
                self.get_logger().info('Failed to push waypoint from the service.')
        except Exception as e:
            self.get_logger().error(f'Error in waypoint_response_callback: {str(e)}')


    def curr_gps_callback(self, msg):
        """
        Use the NavSatFix data to assign value to self.curr_loc
        """
        # assuming the zone_number and zone_letter are the same for the current location and the destination
        self.curr_loc = LatLongPoint(latitude=msg.latitude, longitude=msg.longitude).to_utm()

    def wind_callback(self, msg):
        """
        Use the wind data from msg to assign value to self.wind_dir
        """
        self.wind_dir = msg.data

    def heading_dir_callback(self, msg):
        """
        Use the imu data to assign value to self.heading_dir
        """
        data = msg.z
        # roll_x, roll_y, roll_z = euler_from_quaternion(data.x, data.y, data.z, data.w)
        self.heading_dir = data

    def buoy_position_callback(self, msg):
        return

    # def calculate_rudder_angle(self):
    #     """
    #     The main function for calculating the rudder angle.
    #     This function will do nothing if current location is missing, or the boat is tacking
    #     but the tacking point is missing, or the current destination is missing (inclusive or).
    #     Otherwise, this function will publish the rudder angle, and the rudder angle is 
    #     from -25 to 25 degree rounded to the nearest 5.

    #     Precondition: self.curr_dest is not None, self.curr_loc is not None, self.heading_dir is not None
    #     """
    #     if self.curr_dest is None or self.curr_loc is None or self.heading_dir is None:
    #         return # Not enough information to calculate rudder angle yet

    #     #Handle tacking logic in step
    #     if self.tacking: 
    #         dest = self.tacking_point
    #     else:
    #         dest = self.curr_dest

    #     x_distance = dest.easting - self.curr_loc.easting
    #     y_distance = dest.northing - self.curr_loc.northing

    #     target_bearing = np.arctan2(y_distance, x_distance) * 180 / np.pi
    #     self.get_logger().info(f'Target Bearing: {target_bearing}')

    #     diff = np.mod(self.heading_dir - target_bearing + 180, 360) - 180
    #     self.get_logger().info(f'Heading Difference: {diff}')
    #     self.diff = diff

    #     if(not self.in_nogo()):
    #         rudder_angle = (diff / 180.0) * 25
    #     else:
    #         rudder_angle = np.sign(diff) * 25 # max rudder angle if we are in nogo zone

    #     self.get_logger().info(f'Rudder Angle Raw: {rudder_angle}')

    #     # Assuming rudder_angle is a floating-point number and you want it to be an Int32 message
    #     rudder_angle = np.floor(rudder_angle / 5) * 5  # Floor the angle to the nearest multiple of 5
    #     rudder_angle = int(rudder_angle)  # Convert to int since Int32 requires an integer value

    #     self.get_logger().info(f'Rudder Angle: {rudder_angle}')

    #     # Create an Int32 message and publish the rudder angle
    #     rudder_angle_msg = Int32()
    #     rudder_angle_msg.data = rudder_angle

    #     self.rudder_angle_pub.publish(rudder_angle_msg)
        
    # def in_nogo(self):
    #     """
    #     Check if the boat is in nogo zone based on the wind direction
    #     """
    #     self.get_logger().info(f'Wind Direction: {self.wind_dir}')
    #     if self.wind_dir is None:
    #         return False
    #     return (150 < self.wind_dir < 210)

    # def calculateTP(self) -> UTMPoint:
    #     """
    #     Calcualte tacking point to begin tacking. uses winddir + dest
    #     Assuming that the boat is heading towards the positive x-axis and the destination

    #     Precondition: self.in_nogo() is true. self.tacking is false. self.curr_loc is not None. self.curr_dest is not None. self.wind_dir is not None.
    #     """

    #     assert self.in_nogo(), "Not in nogo zone"
    #     assert self.tacking, "Already tacking"
    #     assert self.curr_loc is not None, "Current location is None"
    #     assert self.curr_dest is not None, "Current destination is None"
    #     assert self.wind_dir is not None, "Wind direction is None"

    #     try:
    #         latlong = self.curr_loc.to_latlon()
    #         lat,long = latlong.latitude, latlong.longitude
    #         self.get_logger().info(f'Current Location: ({lat}, {long})')
    #     except Exception as e:
    #         self.get_logger().error(f'Error in Lat Long: {str(e)}') 

    #     x_distance = self.curr_dest.easting - self.curr_loc.easting
    #     y_distance = self.curr_dest.northing - self.curr_loc.northing

    #     heading_to_dest = np.arctan2(y_distance, x_distance) * 180 / np.pi
    #     tack_diff = np.mod(self.heading_dir - heading_to_dest + 180, 360) - 180

    #     # if we're out of the no-go-zone, don't tack at all
    #     if(abs(tack_diff) > self.no_go_zone):
    #         return self.curr_dest

    #     # tack left or right depending on the angle from the middling line
    #     if(tack_diff > 0):
    #         #tack on right
    #         tack_angle = (self.heading_dir - self.no_go_zone) % 360
    #         approach_angle = (self.no_go_zone + self.heading_dir) % 360

    #     else:
    #         #tack on left
    #         tack_angle = (self.no_go_zone + self.heading_dir) % 360
    #         approach_angle = (self.heading_dir - self.no_go_zone) % 360

    #     # calculate tacking point as intersection of the tack vector and the approach vector
    #     vec1 = np.array([np.cos(np.deg2rad(tack_angle)), np.sin(np.deg2rad(tack_angle))])
    #     vec2 = -np.array([np.cos(np.deg2rad(approach_angle)), np.sin(np.deg2rad(approach_angle))])

    #     self.get_logger().info(f'Vector 1: {vec1}')
    #     self.get_logger().info(f'Vector 2: {vec2}')

    #     P1 = np.array([self.curr_loc.easting, self.curr_loc.northing])
    #     P2 = np.array([self.curr_dest.easting, self.curr_dest.northing])

    #     # intersection of two lines
    #     A = np.column_stack((vec1, -vec2))
    #     b = P2 - P1

    #     t_vals = np.linalg.solve(A, b)

    #     tacking_point = P1 + t_vals[0] * vec1
    #     self.get_logger().info(f'Tacking Point: {tacking_point}')

    #     tp = UTMPoint(easting=tacking_point[0], northing=tacking_point[1], zone_number=self.curr_loc.zone_number, zone_letter=self.curr_loc.zone_letter)
    #     assert tp.easting > 100000 and tp.easting < 900000, "Easting out of range"

    #     # publish new TP if we do not encounter an exception
    #     try:
    #         tacking_point_msg = tp.to_navsatfix_msg()
    #         self.tacking_point_pub.publish(tacking_point_msg)
    #     except Exception as e: 
    #         self.get_logger().error(f'Tacking point easting: {tp.easting}, northing: {tp.northing}')
    #         self.get_logger().error(f'Error in calculateTP: {str(e)}') 

    #     self.get_logger().info(f'Tacking Point: {str(tp.to_latlon())}')

    #     return tp

    # def step(self):
    #     """
    #     Sail. 
    #     """
    #     if self.curr_loc is None or (self.tacking and self.tacking_point is None) or self.curr_dest is None:
    #         # Not enough information to calculate rudder angle yet
    #         return

    #     if self.in_nogo() and not self.tacking:
    #         self.get_logger().info("Beginning Tacking")
    #         self.tacking = True
    #         self.tacking_point = self.calculateTP()
    #         self.tack_time_tracker = 0
    #     elif self.tacking:
    #         self.tack_time_tracker += self.timer_period
    #         if self.tack_time_tracker >= self.tacking_buffer:
    #             self.get_logger().info("End Tack")

    #             self.tacking = False
    #             self.tacking_point = None
    #     self.calculate_rudder_angle()

    #     self.get_logger().info("Sailing")
    
    def initialize_search_pattern(self):
        search_pattern = []
        expansion_step = 5
        max_radius = 100

        # Start at the middle top
        x = 0
        direction = 1

        while x <= max_radius:
            search_pattern.append([x, direction * max_radius])
            direction *= -1
            x += expansion_step
        while x >= -1 * max_radius:
            search_pattern.append([x, direction * max_radius])
            direction *= -1
            x -= expansion_step
        
        if self.wind_dir is not None and self.curr_loc is not None:
            rotation_angle = math.radians(self.wind_dir)
            translation = self.curr_loc.easting, self.curr_loc.northing
            zone_number = self.curr_loc.zone_number
            zone_letter = self.curr_loc.zone_letter
        else:
            self.wind_dir = 0
            self.curr_loc = LatLongPoint(42.443962, -76.501884).to_utm()
            rotation_angle = math.radians(self.wind_dir)
            translation = self.curr_loc.easting, self.curr_loc.northing
            zone_number = self.curr_loc.zone_number
            zone_letter = self.curr_loc.zone_letter
            self.get_logger().error(f'Search pattern failed to initialize.')

        for i in range(len(search_pattern)):
            x, y = search_pattern[i]
            easting = x * math.cos(rotation_angle) - y * math.sin(rotation_angle)
            northing = x * math.sin(rotation_angle) + y * math.cos(rotation_angle)
            easting += translation[0]
            northing += translation[1]

            new_waypoint = UTMPoint(easting, northing, zone_number, zone_letter).to_latlon()
    
            search_pattern[i] = (new_waypoint.latitude, new_waypoint.longitude)
        
            self.set_waypoints(search_pattern)

def main(args=None):
    rclpy.init(args=args)

    buoy_search = BuoySearch()


    # Explicitly destroy the node when done
    try:
        rclpy.spin(buoy_search)
    except KeyboardInterrupt:
        pass


    buoy_search.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

