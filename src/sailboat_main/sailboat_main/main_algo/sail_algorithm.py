import numpy as np
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from rclpy.task import Future
from typing import Optional

from sailboat_interface.srv import Waypoint
from sailboat_interface.msg import AlgoDebug
from Points import LatLongPoint, UTMPoint
from states import SailState

class MainAlgo(Node):
    """
    The sailing algorithm responsible for changing the rudder angle based on the 
    current location, destination, and heading direction.
    """
    # Sensor Data
    wind_direction : Optional[float]
    current_location : Optional[UTMPoint]
    current_waypoint : Optional[UTMPoint]
    heading_direction : Optional[float]
    heading_difference : Optional[float]
    dist_to_dest : Optional[float]

    # Algorithm State
    state : SailState
    current_destination : Optional[UTMPoint]
    tacking_point : Optional[UTMPoint]
    jibing_point : Optional[UTMPoint]

    # Algorithm Parameters (angles calculated in degrees in one direction symmetric around the centerline)
    tacking_buffer : int # cooldown between calculating tacking points
    tack_no_go_zone : int # anglular size of the no-go-zone on one side of the boat's centerline
    jibe_danger_zone: int # distance to the waypoint at which we should start jibing
    jibe_angle : int
    neutral_zone: int
    dist_to_dest : Optional[float]

    # Physical Parameters
    MAX_RUDDER_ANGLE : int = 25 # max rudder angle in degrees
    NEUTRAL_RUDDER_ANGLE : int = 0 

    def __init__(self):
        super().__init__('main_algo')

        self.declare_parameter('timer_period', 0.500) 
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('tacking_buffer', 15)
        self.tacking_buffer = self.get_parameter('tacking_buffer').value

        self.declare_parameter('gybe_buffer', 15)
        self.gybe_buffer = self.get_parameter('gybe_buffer').value

        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value
    
        self.declare_parameter("no_go_zone", 45)
        self.no_go_zone = self.get_parameter("no_go_zone").value

        self.declare_parameter("danger_zone", 10)
        self.danger_zone = self.get_parameter("danger_zone").value

        self.tack_time_tracker = 0
        self.gybe_time_tracker = 0

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
        
        self.timer = self.create_timer(self.timer_period, self.step)

        # Publisher for rudder angle
        self.rudder_angle_pub = self.create_publisher(Int32, 'algo_rudder', 10)

        self.tacking_point_pub = self.create_publisher(NavSatFix, 'tacking_point', 10)

        # Internal state
        self.wind_direction = None
        self.curr_loc = None
        self.tacking_point = None
        self.heading_direction = None
        self.curr_dest = None
        self.heading_difference = None
        self.dist_to_dest = None

        if self.debug:
            # Publisher for internal state
            self.state_pub = self.create_publisher(AlgoDebug, 'main_algo_debug', 10)
            # Timer to publish state every 1 second
            self.state_timer = self.create_timer(1.0, self.publish_state_debug)

        # Publisher for tacking point
        self.tacking_point_pub = self.create_publisher(NavSatFix, 'tacking_point', 10)

        self.get_logger().info('Main-algo started successfully')

    def step(self):
        """
        Sail. 
        """
        if self.curr_loc is None or self.curr_dest is None:
            # Not enough information to calculate rudder angle yet
            return
        
        self.update_state()

        # update heading difference: heading_direction - target_bearing
        self.heading_difference = np.mod(self.heading_direction -
                                         self.current_location.target_bearing_to(self.current_destination) + 180, 360) - 180
        self.get_logger().info(f'Heading Difference: {self.heading_difference}')

        if self.sail_state == SailState.NORMAL:
            self.set_normal_rudder()
        elif self.sail_state == SailState.TACK:
            self.set_tacking_rudder()
        elif self.sail_state == SailState.JIBE:
            self.set_normal_rudder() # TODO: do we need to set a different (smaller) rudder angle?
            self.notify_trim_sail() # TODO: implement this function + should notify every step maybe
        else:
            raise ValueError("Unknown state") 

    def update_state(self):
        """
        Update the state of the algorithm. This function is called when the state changes.
        """
        if self.sail_state == SailState.NORMAL:
            self.get_logger().info("Normal sailing")
            if self.waypoint_in_no_go_zone():
                self.sail_state = SailState.TACK
                self.curr_dest = self.calculate_tacking_point()
            elif self.waypoint_in_danger_zone():
                self.curr_dest = self.calculate_jibing_point()
                self.sail_state = SailState.JIBE
            else:
                self.curr_dest = self.current_waypoint

        elif self.sail_state == SailState.TACK:
            self.get_logger().info("Tacking")
            # if we reached the tacking point, switch to normal sailing
            if self.current_location.distance_to(self.curr_dest) < 5:
                self.get_logger().info("Reached tacking point")
                self.curr_dest = self.current_waypoint
                self.sail_state = SailState.NORMAL
            elif self.tack_time_tracker > self.tacking_buffer:
                # TODO: do we need to exit the no go zone to gain speed before recalculating?
                self.curr_dest = self.calculate_tacking_point()
                self.sail_state = SailState.TACK
            else:
                pass # keep tacking
            
        elif self.sail_state == SailState.JIBE:
            self.get_logger().info("Jibing")
            # if we reached the jibing point, switch to normal sailing
            if self.current_location.distance_to(self.curr_dest) < 5:
                self.get_logger().info("Reached jibe point")
                self.sail_state = SailState.NORMAL
            else:
                pass # keep jibing
            
        else:
            self.get_logger().info("Unknown state")

        # TODO: if the boat is in the danger zone, we should inform the sail to trim

    def set_tacking_rudder(self): 
        """""""""
            Tacks the sail when called, setting rudder angle to the max rudder angle on the correct side of the tack.
        """
        #tack on right or left 
        # TODO: is this correct to use self.heading_difference >= 0
        if np.absolute(self.heading_difference) <= self.neutral_zone:
            rudder_angle = self.NEUTRAL_RUDDER_ANGLE
        else:
            rudder_angle = self.MAX_RUDDER_ANGLE if self.heading_difference >= 0 else -self.MAX_RUDDER_ANGLE
        
        #publish 
        self.get_logger().info(f'Rudder Angle: {rudder_angle}')
        rudder_angle_msg = Int32()
        rudder_angle_msg.data = rudder_angle
        self.rudder_angle_pub.publish(rudder_angle_msg)

    def set_normal_rudder(self):
        """
            Sails normally to REACHABLE destination (assumes not in no-go etc.)
        """
        rudder_angle = self.heading_difference / 180 * 25
        rudder_angle = int(rudder_angle // 5 * 5)  # round to nearest 5 degrees

        self.get_logger().info(f'Rudder Angle: {rudder_angle}')

        # Create an Int32 message and publish the rudder angle
        rudder_angle_msg = Int32()
        rudder_angle_msg.data = rudder_angle
        self.rudder_angle_pub.publish(rudder_angle_msg)
    
    def calculate_jibing_point(self) -> UTMPoint:
        """
        Calculate the jibe point to begin jibing. uses winddir + dest
        """
        assert self.waypoint_in_danger_zone(), "Not in nogo zone"
        assert self.state is SailState.JIBE, "Already jibe"
        assert self.curr_loc is not None, "Current location is None"
        assert self.curr_dest is not None, "Current destination is None"
        assert self.wind_direction is not None, "Wind direction is None"

        try:
            latlong = self.curr_loc.to_latlon()
            lat,long = latlong.latitude, latlong.longitude
            self.get_logger().info(f'Current Location: ({lat}, {long})')
        except Exception as e:
            self.get_logger().error(f'Error in Lat Long: {str(e)}') 

        x_distance = self.curr_dest.easting - self.curr_loc.easting
        y_distance = self.curr_dest.northing - self.curr_loc.northing

        heading_to_dest = np.arctan2(y_distance, x_distance) * 180 / np.pi
        jibe_diff = np.mod(self.heading_direction - heading_to_dest + 180, 360) - 180


        if(abs(jibe_diff) > self.jibe_danger_zone):
            return self.curr_dest

        if(jibe_diff > 0):
            #tack on right
            jibe_angle = (self.heading_direction - self.jibe_danger_zone) % 360
            approach_angle = (self.jibe_danger_zone + self.heading_direction) % 360
        else:
            #tack on left
            jibe_angle = (self.jibe_danger_zone + self.heading_direction) % 360
            approach_angle = (self.heading_direction - self.jibe_danger_zone) % 360

        vec1 = np.array([np.cos(np.deg2rad(jibe_angle)), np.sin(np.deg2rad(jibe_angle))])
        vec2 = -np.array([np.cos(np.deg2rad(approach_angle)), np.sin(np.deg2rad(approach_angle))])

        self.get_logger().info(f'Vector 1: {vec1}')
        self.get_logger().info(f'Vector 2: {vec2}')

        P1 = np.array([self.curr_loc.easting, self.curr_loc.northing])
        P2 = np.array([self.curr_dest.easting, self.curr_dest.northing])

        # intersection of two lines
        A = np.column_stack((vec1, -vec2))
        b = P2 - P1

        t_vals = np.linalg.solve(A, b)

        jibing_point = P1 + t_vals[0] * vec1
        self.get_logger().info(f'Jibe Point: {jibing_point}')

        gp = UTMPoint(easting=jibing_point[0], northing=jibing_point[1], 
                      zone_number=self.curr_loc.zone_number, zone_letter=self.curr_loc.zone_letter)

        assert gp.easting > 100000 and gp.easting < 900000, "Easting out of range"

        # publish new GP if we do not encounter an exception
        try:
            gybe_point_msg = gp.to_navsatfix_msg()
            self.tacking_point_pub.publish(gybe_point_msg)
        except Exception as e: 
            self.get_logger().error(f'Gybing point easting: {gp.easting}, northing: {gp.northing}')
            self.get_logger().error(f'Error in calculateGP: {str(e)}') 

        self.get_logger().info(f'Gybe Point: {str(gp.to_latlon())}')

        return gp

    def calculate_tacking_point(self) -> UTMPoint:
        """
        Calcualte tacking point to begin tacking. uses winddir + dest
        Assuming that the boat is heading towards the positive x-axis and the destination

        Precondition: self.waypoint_in_nogo_zone() is true. self.tacking is false. self.curr_loc is not None. self.curr_dest is not None. self.wind_direction is not None.
        """

        assert self.waypoint_in_no_go(), "Waypoint not in nogo zone"
        assert self.state is SailState.TACK, "Already tacking"
        assert self.curr_loc is not None, "Current location is None"
        assert self.curr_dest is not None, "Current destination is None"
        assert self.wind_direction is not None, "Wind direction is None"

        try:
            latlong = self.curr_loc.to_latlon()
            lat,long = latlong.latitude, latlong.longitude
            self.get_logger().info(f'Current Location: ({lat}, {long})')
        except Exception as e:
            self.get_logger().error(f'Error in Lat Long: {str(e)}') 

        x_distance = self.curr_dest.easting - self.curr_loc.easting
        y_distance = self.curr_dest.northing - self.curr_loc.northing

        heading_to_dest = np.arctan2(y_distance, x_distance) * 180 / np.pi
        tack_diff = np.mod(self.heading_direction - heading_to_dest + 180, 360) - 180

        # tack left or right depending on the angle from the middling line
        if(tack_diff > 0):
            #tack on right
            tack_angle = (self.heading_direction - self.no_go_zone) % 360
            approach_angle = (self.no_go_zone + self.heading_direction) % 360

        else:
            #tack on left
            tack_angle = (self.no_go_zone + self.heading_direction) % 360
            approach_angle = (self.heading_direction - self.no_go_zone) % 360

        # calculate tacking point as intersection of the tack vector and the approach vector
        vec1 = np.array([np.cos(np.deg2rad(tack_angle)), np.sin(np.deg2rad(tack_angle))])
        vec2 = -np.array([np.cos(np.deg2rad(approach_angle)), np.sin(np.deg2rad(approach_angle))])

        self.get_logger().info(f'Vector 1: {vec1}')
        self.get_logger().info(f'Vector 2: {vec2}')

        P1 = np.array([self.curr_loc.easting, self.curr_loc.northing])
        P2 = np.array([self.curr_dest.easting, self.curr_dest.northing])

        # intersection of two lines
        A = np.column_stack((vec1, -vec2))
        b = P2 - P1

        t_vals = np.linalg.solve(A, b)

        tacking_point = P1 + t_vals[0] * vec1
        self.get_logger().info(f'Tacking Point: {tacking_point}')

        tp = UTMPoint(easting=tacking_point[0], northing=tacking_point[1], 
                      zone_number=self.curr_loc.zone_number, zone_letter=self.curr_loc.zone_letter)
        assert tp.easting > 100000 and tp.easting < 900000, "Easting out of range"

        # publish new TP if we do not encounter an exception
        try:
            tacking_point_msg = tp.to_navsatfix_msg()
            self.tacking_point_pub.publish(tacking_point_msg)
        except Exception as e: 
            self.get_logger().error(f'Tacking point easting: {tp.easting}, northing: {tp.northing}')
            self.get_logger().error(f'Error in calculateTP: {str(e)}') 

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
        return (150 < self.wind_direction < 210)
    
    def boat_in_danger_zone(self):
        """
        Check if the boat is in danger zone based on the wind direction
        """
        self.get_logger().info(f'Wind Direction: {self.wind_direction}')
        if self.wind_direction is None:
            return False
        return (self.wind_direction > 210 and self.wind_direction < 330)
    
    def waypoint_in_danger_zone(self) -> bool:
        """
        Check if the current destination waypoint is in the danger zone.
        """
        if self.curr_dest is None or self.curr_loc is None:
            return False

        dist_to_dest = self.curr_loc.distance_to(self.curr_dest)
        self.get_logger().info(f'Distance to destination: {dist_to_dest}')
        return dist_to_dest < self.danger_zone

    def waypoint_in_no_go_zone(self) -> bool:
        """
        Check if the current destination waypoint is in the no-go zone.
        """
        if self.curr_dest is None or self.curr_loc is None:
            return False

        x_distance = self.curr_dest.easting - self.curr_loc.easting
        y_distance = self.curr_dest.northing - self.curr_loc.northing

        heading_to_dest = np.arctan2(y_distance, x_distance) * 180 / np.pi
        tack_diff = np.mod(self.heading_direction - heading_to_dest + 180, 360) - 180

        return abs(tack_diff) < self.no_go_zone


    # ========================= Callbacks =========================
        


    def current_waypoint_callback(self, msg):
        """
        Callback to update the current destination waypoint from the 'current_waypoint' topic.
        """
        self.current_waypoint = LatLongPoint(msg.latitude, msg.longitude).to_utm()
        self.get_logger().info(f'Updated current waypoint to: ({msg.latitude}, {msg.longitude})')

    def publish_state_debug(self):
        """
        Publish the internal state as a JSON string.
        """
        debug_msg = AlgoDebug()
        debug_msg.tacking = self.tacking

        if self.tacking_point is not None:
            debug_msg.tacking_point = self.tacking_point.to_navsatfix_msg()
        else:
            debug_msg.tacking_point = NavSatFix() # empty msg

        debug_msg.heading_direction = Int32()
        debug_msg.heading_direction.data = int(self.heading_direction) if self.heading_direction is not None else 0

        if self.curr_dest is not None:
            debug_msg.curr_dest = self.curr_dest.to_navsatfix_msg()
        else:
            debug_msg.curr_dest = NavSatFix() # empty msg

        debug_msg.heading_difference = Int32()
        debug_msg.heading_difference.data = int(self.heading_difference) if self.diff is not None else 0

        if self.dist_to_dest is not None:
            debug_msg.dist_to_dest = Int32()
            debug_msg.dist_to_dest.data = int(self.dist_to_dest)
        else:
            debug_msg.dist_to_dest = Int32()
            debug_msg.dist_to_dest.data = -1 # Default value if distance is not available

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
        Use the NavSatFix data to assign value to self.curr_loc
        """
        # assuming the zone_number and zone_letter are the same for the current location and the destination
        self.curr_loc = LatLongPoint(latitude=msg.latitude, longitude=msg.longitude).to_utm()

        # Update the distance to destination, check if we have reached our waypoint
        if self.curr_dest is not None:
            self.dist_to_dest = self.current_location.distance_to(self.current_waypoint)
            self.get_logger().info(f'Distance to destination: {self.dist_to_dest}')
            # if we have reached our waypoint, pop it off 
            if self.dist_to_dest < 5:
                self.get_logger().info('=============================== Waypoint popped ===============================')
                self.pop_waypoint()
                self.curr_dest = None

        self.step()

    def wind_callback(self, msg):
        """
        Use the wind data from msg to assign value to self.wind_direction
        """
        self.wind_direction = msg.data

    def heading_direction_callback(self, msg):
        """
        Use the imu data to assign value to self.heading_direction
        """
        data = msg.z
        self.heading_direction = data
        
def main(args=None):
    rclpy.init(args=args)
    main_algo = MainAlgo()
    # Explicitly destroy the node when done
    try:
        rclpy.spin(main_algo)
    except KeyboardInterrupt:
        pass
    main_algo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

