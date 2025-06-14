import numpy as np
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from rclpy.task import Future
from typing import Optional

from sailboat_interface.srv import Waypoint
from sailboat_interface.msg import AlgoDebug
# from .points import LatLongPoint, UTMPoint
from .states import SailState

import utm
import math
# import numpy as np
# from sensor_msgs.msg import NavSatFix

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
    
    def target_bearing_to(self, other: 'UTMPoint') -> float:
        """
        Calculate the bearing to another UTM point.
        """
        assert self.zone_number == other.zone_number, "Zone numbers must be the same for bearing calculation"
        delta_easting = other.easting - self.easting
        delta_northing = other.northing - self.northing
        return np.arctan2(delta_northing, delta_easting) * 180 / np.pi

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

class Algo(Node):
    """
    The sailing algorithm responsible for changing the rudder angle based on the 
    current location, destination, and heading direction.
    """
    # Sensor Data
    wind_direction : Optional[float] = None
    current_location : Optional[UTMPoint] = None
    current_waypoint : Optional[UTMPoint] = None
    heading_direction : Optional[float] = None
    heading_difference : Optional[float] = None
    dist_to_dest : Optional[float] = None

    # Algorithm State
    state : SailState = SailState.NORMAL
    current_destination : Optional[UTMPoint] = None
    tacking_point : Optional[UTMPoint] = None
    current_mode : str = 'manual'

    # Algorithm Parameters (angles calculated in degrees in one direction symmetric around the centerline)
    tacking_buffer : int = 30 
    tack_no_go_zone : int = 60 # anglular size of the no-go-zone on one side of the boat's centerline
    neutral_zone: int = 10 

    # Physical Parameters
    MAX_RUDDER_ANGLE : int = 25 # max rudder angle in degrees
    NEUTRAL_RUDDER_ANGLE : int = 0 
    POP_RADIUS : int = 5 # radius in meters to pop the waypoint

    def __init__(self):
        super().__init__('main_algo')

        self.declare_parameter('timer_period', 0.200) 
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('tacking_buffer', 15)
        self.tacking_buffer = self.get_parameter('tacking_buffer').value
    
        self.declare_parameter("no_go_zone", 60)
        self.no_go_zone = self.get_parameter("no_go_zone").value

        self.declare_parameter("danger_zone", 10)
        self.danger_zone = self.get_parameter("danger_zone").value

        self.tack_time_tracker = 0

        self.sail_state = SailState.NORMAL

        #Subscription for current location
        self.subscription_curr_loc = self.create_subscription(
            NavSatFix,
            '/gps',
            self.curr_gps_callback,
            10)

        #Subscription for heading direction
        self.subscription_heading_direction = self.create_subscription(
            Vector3,
            '/imu',
            self.heading_direction_callback,
            10)

        #Subscription for wind direction
        self.subscription_wind_direction = self.create_subscription(
            Int32,
            'wind',
            self.wind_callback,
            10)

        #Subscription for current_waypoint
        self.subscription_current_waypoint = self.create_subscription(
            NavSatFix,
            'current_waypoint',
            self.current_waypoint_callback,
            10)
        
        #Subscription for runtime no-go zone adjustments
        self.subscription_no_go_zone = self.create_subscription(
            Int32, 
            'no_go_zone',
            self.no_go_zone_callback,
            10)
        
        #Subscription for runtime neutral zone adjustments
        self.subscription_neutral_zone = self.create_subscription(
            Int32, 
            'neutral_zone',
            self.neutral_zone_callback,
            10)
        
        #Subscription for runtime neutral zone adjustments
        self.subscription_tacking_buffer = self.create_subscription(
            Int32, 
            'tacking_buffer',
            self.tacking_buffer_callback,
            10)
        
        # Subscription for the mode 
        self.mode_subscription = self.create_subscription(
            String,
            'current_mode',
            self.mode_callback,
            10)
        
        # Publisher for rudder angle
        self.rudder_angle_pub = self.create_publisher(Int32, 'algo_rudder', 10)

        # Publisher for danger zone notification
        self.danger_zone_pub = self.create_publisher(Bool, 'danger_zone', 10)

        # Setup primary loop for stepping through sailing algorithm
        self.step_timer = self.create_timer(self.timer_period, self.step)

        # Handle Debug Publishing
        self.declare_parameter("debug", True)
        self.debug = self.get_parameter("debug").value
        self.get_logger().info(f'Debug mode: {self.debug}')
        if self.debug:
            # Publisher for internal state
            self.state_pub = self.create_publisher(AlgoDebug, 'main_algo_debug', 10)
            # Timer to publish state every 1 second
            self.state_timer = self.create_timer(1.0, self.publish_state_debug)

        self.get_logger().info('Main-algo started successfully')

    def step(self):
        """
        Sail. 
        """
        if self.current_location is None or self.current_waypoint is None or self.heading_direction is None:
            # Not enough information to calculate rudder angle yet
            return
        
        # on the first iteration, set state to NORMAL and destination to current waypoint
        if self.current_destination is None:
            self.current_destination = self.current_waypoint
            self.sail_state = SailState.NORMAL
        
        self.update_state()

        # update heading difference: heading_direction - target_bearing
        self.heading_difference = np.mod(self.heading_direction -
                                         self.current_location.target_bearing_to(self.current_destination) + 180, 360) - 180

        if self.sail_state == SailState.NORMAL:
            self.set_normal_rudder()
        elif self.sail_state == SailState.TACK:
            self.set_tacking_rudder()
        else:
            raise ValueError("Unknown state") 

    def update_state(self):
        """
        Update the state of the algorithm. This function is called when the state changes.
        """
        if self.sail_state == SailState.NORMAL:
            if self.waypoint_in_nogo_zone():
                self.current_destination = self.calculate_tacking_point()
                self.tack_time_tracker = 0
                self.sail_state = SailState.TACK
            else:
                pass
        elif self.sail_state == SailState.TACK:
            # if we reached the tacking point, switch to normal sailing
            if self.current_location.distance_to(self.current_destination) < self.POP_RADIUS:
                self.get_logger().info("Reached tacking point")
                self.current_destination = self.current_waypoint
                self.sail_state = SailState.NORMAL
            elif self.tack_time_tracker > self.tacking_buffer:
                if not self.waypoint_in_nogo_zone():
                    self.current_destination = self.current_waypoint
                    self.sail_state = SailState.NORMAL
                else:
                    self.current_destination = self.calculate_tacking_point()
                    self.tack_time_tracker = 0 # reset time tracker
                    self.sail_state = SailState.TACK
            else:
                self.tack_time_tracker += self.timer_period # keep tacking
            
        else:
            self.get_logger().info("Unknown state")


    def set_tacking_rudder(self): 
        """""""""
            Tacks the sail when called, setting rudder angle to the max rudder angle on the correct side of the tack.
        """
        #tack on right or left 
        if np.absolute(self.heading_difference) <= self.neutral_zone:
            rudder_angle = np.floor(self.heading_difference / 180 * 25)
        else:
            rudder_angle = self.MAX_RUDDER_ANGLE if self.heading_difference >= 0 else -self.MAX_RUDDER_ANGLE
        
        #publish 
        self.get_logger().info(f'Rudder Angle: {rudder_angle}')
        rudder_angle_msg = Int32()
        rudder_angle_msg.data = int(rudder_angle)
        self.rudder_angle_pub.publish(rudder_angle_msg)

    def set_normal_rudder(self):
        """
            Sails normally to REACHABLE destination (assumes not in no-go etc.)
        """
        if(not self.boat_in_nogo_zone()):
            rudder_angle = np.round(self.heading_difference / 180 * 25)
        else: # this handles the case where we need to push out of the no-go zone but do not need a tacking point
            rudder_angle = self.MAX_RUDDER_ANGLE if self.heading_difference >= 0 else -self.MAX_RUDDER_ANGLE

        # rudder_angle = int(rudder_angle // 5 * 5)  # round to nearest 5 degrees

        self.get_logger().info(f'Rudder Angle: {rudder_angle}')

        # Create an Int32 message and publish the rudder angle
        rudder_angle_msg = Int32()
        rudder_angle_msg.data = int(rudder_angle)
        self.rudder_angle_pub.publish(rudder_angle_msg)
    

    def calculate_tacking_point(self) -> UTMPoint:
        """
        Calcualte tacking point to begin tacking. uses winddir + dest
        Assuming that the boat is heading towards the positive x-axis and the destination
        """
        assert self.current_location is not None, "Current location is None"
        assert self.current_destination is not None, "Current destination is None"
        assert self.wind_direction is not None, "Wind direction is None"
        assert self.absolute_wind_dir is not None, "Absolute wind direction is None"

        try:
            latlong = self.current_location.to_latlon()
            lat,long = latlong.latitude, latlong.longitude
            self.get_logger().info(f'Current Location: ({lat}, {long})')
        except Exception as e:
            self.get_logger().error(f'Error in Lat Long: {str(e)}') 

        #wind difference: angle between the absolute wind direction and the target waypoint. 
        opposite_wind_dir = (self.absolute_wind_dir + 180) % 360 # the angles work out due to symmetry
        wind_diff = ( self.current_location.target_bearing_to(self.current_waypoint) - opposite_wind_dir + 180) % 360 - 180

        self.get_logger().info(f'Wind Difference: {wind_diff}')
        # tack left or right depending on the angle from the middling line
        if(wind_diff < 0):
            #tack on right
            tack_angle = (self.absolute_wind_dir- self.no_go_zone) % 360
            approach_angle = (self.no_go_zone + self.absolute_wind_dir) % 360

        else:
            #tack on left
            tack_angle = (self.no_go_zone + self.absolute_wind_dir) % 360
            approach_angle = (self.absolute_wind_dir- self.no_go_zone) % 360

        # calculate tacking point as intersection of the tack vector and the approach vector
        vec1 = np.array([np.cos(np.deg2rad(tack_angle)), np.sin(np.deg2rad(tack_angle))])
        vec2 = -np.array([np.cos(np.deg2rad(approach_angle)), np.sin(np.deg2rad(approach_angle))])

        P1 = np.array([self.current_location.easting, self.current_location.northing])
        P2 = np.array([self.current_waypoint.easting, self.current_waypoint.northing])

        # intersection of two lines
        A = np.column_stack((vec1, -vec2))
        b = P2 - P1

        t_vals = np.linalg.solve(A, b)

        tacking_point = P1 + t_vals[0] * vec1

        tp = UTMPoint(easting=tacking_point[0], northing=tacking_point[1], 
                      zone_number=self.current_location.zone_number, zone_letter=self.current_location.zone_letter)
        assert tp.easting > 100000 and tp.easting < 900000, "Easting out of range"

        self.get_logger().info(f'Tacking Point: {str(tp.to_latlon())}')

        return tp

    # ======================== Helper Functions =======================
        
    def boat_in_nogo_zone(self):
        """
        Check if the boat is in nogo zone based on the wind direction
        """
        self.get_logger().info(f'Wind Direction: {self.wind_direction}')
        if self.wind_direction is None:
            return False
        return (180 - self.no_go_zone < self.wind_direction < 180 + self.no_go_zone)

    def waypoint_in_nogo_zone(self):
        """
        Check if the current waypoint is in the no-go zone based on the wind direction
        """
        if self.current_waypoint is None or self.wind_direction is None or self.absolute_wind_dir is None:
            return False

        target_bearing = self.current_location.target_bearing_to(self.current_waypoint)
        opposite_wind_dir = (self.absolute_wind_dir + 180) % 360 # the angles work out dw

        diff = (target_bearing - opposite_wind_dir + 180) % 360 - 180
        return abs(diff) < self.no_go_zone    
    
# ========================= Callbacks & Publishers =========================

    def current_waypoint_callback(self, msg):
        """
        Callback to update the current destination waypoint from the 'current_waypoint' topic.
        """
        self.current_waypoint = LatLongPoint(msg.latitude, msg.longitude).to_utm()
        self.new_waypoint_flag = True
        self.get_logger().info(f'Updated current waypoint to: ({msg.latitude}, {msg.longitude})')

        # reset entire state
        self.get_logger().info("New waypoint received")
        self.new_waypoint_flag = False
        self.current_destination = self.current_waypoint
        self.tack_time_tracker = 0
        self.jibe_time_tracker = 0
        self.sail_state = SailState.NORMAL
        

    def publish_state_debug(self):
        """
        Publish the internal state as a JSON string.
        """
        self.get_logger().info('Publishing internal state')
        debug_msg = AlgoDebug()
        # update to handle jibing  
        debug_msg.tacking = True if self.sail_state == SailState.TACK else False

        debug_msg.heading_dir = Int32()
        debug_msg.heading_dir.data = int(self.heading_direction) if self.heading_direction is not None else 0

        if self.current_destination is not None:
            debug_msg.curr_dest = self.current_destination.to_navsatfix_msg()
        else:
            self.get_logger().info("Current destination is None")
            debug_msg.curr_dest = NavSatFix() # empty msg

        debug_msg.diff = Int32()
        debug_msg.diff.data = int(self.heading_difference) if self.heading_difference is not None else 0

        if self.dist_to_dest is not None:
            debug_msg.dist_to_dest = Int32()
            debug_msg.dist_to_dest.data = int(self.dist_to_dest)
        else:
            debug_msg.dist_to_dest = Int32()
            debug_msg.dist_to_dest.data = -1 # Default value if distance is not available

        debug_msg.no_go_zone = Int32()
        debug_msg.no_go_zone.data = int(self.no_go_zone)

        debug_msg.neutral_zone = Int32()
        debug_msg.neutral_zone.data = int(self.neutral_zone)
        
        self.state_pub.publish(debug_msg)

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

    def curr_gps_callback(self, msg):
        """
        Use the NavSatFix data to assign value to self.current_location
        """
        # assuming the zone_number and zone_letter are the same for the current location and the destination
        self.current_location = LatLongPoint(latitude=msg.latitude, longitude=msg.longitude).to_utm()

        # Update the distance to destination, check if we have reached our waypoint
        if self.current_destination is not None:
            self.dist_to_dest = self.current_location.distance_to(self.current_waypoint)
            self.get_logger().info(f'Distance to destination: {self.dist_to_dest}')
            # if we have reached our waypoint, pop it off 
            if self.dist_to_dest < 5:
                if self.current_mode != 'station_keeping':
                    self.get_logger().info('=============================== Waypoint popped ===============================')
                    self.pop_waypoint()
                    self.current_destination = None

    def wind_callback(self, msg):
        """
        Use the wind data from msg to assign value to self.wind_direction
        """
        self.wind_direction = msg.data
        if self.heading_direction is not None:
            self.absolute_wind_dir = (self.wind_direction + self.heading_direction) % 360
            self.get_logger().info(f'Wind Direction: {self.wind_direction}, Absolute Wind Direction: {self.absolute_wind_dir}')

    def heading_direction_callback(self, msg):
        """
        Use the imu data to assign value to self.heading_direction
        """
        self.get_logger().info(f'Heading Direction: {msg.z}')
        data = msg.z
        self.heading_direction = data

    def no_go_zone_callback(self, msg):
        """
        Use the no-go zone data from msg to assign value to self.no_go_zone
        """
        self.no_go_zone = msg.data
        self.get_logger().info(f'No-Go Zone: {self.no_go_zone}')

    def neutral_zone_callback(self, msg):
        """
        Use the neutral zone data from msg to assign value to self.neutral_zone
        """
        self.neutral_zone = msg.data
        self.get_logger().info(f'Neutral Zone: {self.neutral_zone}')

    def tacking_buffer_callback(self, msg):
        """
        TODO: add a description for this callback
        """
        self.tacking_buffer = msg.data
        self.get_logger().info(f'Tacking Buffer: {self.tacking_buffer}')

    def mode_callback(self, msg):
        """
        Callback to update the current mode of the algorithm.
        """
        self.current_mode = msg.data      

def main(args=None):
    rclpy.init(args=args)
    main_algo = Algo()
    # Explicitly destroy the node when done
    try:
        rclpy.spin(main_algo)
    except KeyboardInterrupt:
        pass
    main_algo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
