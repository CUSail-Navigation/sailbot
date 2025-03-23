import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.service import Service

import utm
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32
from rclpy.task import Future

from sailboat_interface.srv import Waypoint
from sailboat_interface.msg import AlgoDebug
from time import sleep


import math

class MainAlgo(Node):
    """
    The sailing algorithm responsible for changing the rudder angle based on the 
    current location, destination, and heading direction.
    """
    
    def __init__(self):
        super().__init__('main_algo')

        self.declare_parameter('timer_period', 0.500) 
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('tacking_buffer', 15)
        self.tacking_buffer = self.get_parameter('tacking_buffer').value

        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value

        self.tack_time_tracker = 0

        #Subscription for current location
        self.subscription_curr_loc = self.create_subscription(
            NavSatFix,
            '/gps',
            self.curr_gps_callback,
            10)

        #Subscription for heading direction
        self.subscription_heading_dir = self.create_subscription(
            Imu,
            '/imu',
            self.heading_dir_callback,
            10)

        #Subscription for wind direction
        self.subscription_wind_dir = self.create_subscription(
            Int32,
            'wind',
            self.wind_callback,
            10)

        self.timer = self.create_timer(self.timer_period, self.step)

        # Publisher for rudder angle
        self.rudder_angle_pub = self.create_publisher(Int32, 'algo_rudder', 10)
        self.tacking_point_pub = self.create_publisher(NavSatFix, 'tacking_point', 10)

        # Internal state
        self.wind_dir = None
        self.curr_loc = None
        self.tacking = False
        self.tacking_point = None
        self.heading_dir = None
        self.curr_dest = None
        self.zone_number = None
        self.zone_letter = None
        self.diff = None

        if self.debug:
            # Publisher for internal state
            self.state_pub = self.create_publisher(AlgoDebug, 'main_algo_debug', 10)
            # Timer to publish state every 1 second
            self.state_timer = self.create_timer(1.0, self.publish_state_debug)

        self.request_new_waypoint()
        self.get_logger().info('Main-algo started successfully')  # Check if this line prints

    def publish_state_debug(self):
        """
        Publish the internal state as a JSON string.
        """
        debug_msg = AlgoDebug()
        debug_msg.tacking = self.tacking

        if self.tacking_point is not None:
            debug_msg.tacking_point = NavSatFix()
            debug_msg.tacking_point.latitude, debug_msg.tacking_point.longitude = utm.to_latlon(
            self.tacking_point.x, self.tacking_point.y, self.zone_number, self.zone_letter
            )
        else:
            debug_msg.tacking_point = NavSatFix()

        debug_msg.heading_dir = Int32()
        debug_msg.heading_dir.data = int(self.heading_dir) if self.heading_dir is not None else 0

        if self.curr_dest is not None:
            debug_msg.curr_dest = NavSatFix()
            debug_msg.curr_dest.latitude, debug_msg.curr_dest.longitude = utm.to_latlon(
            self.curr_dest.x, self.curr_dest.y, self.zone_number, self.zone_letter
            )
        else:
            debug_msg.curr_dest = NavSatFix()

        debug_msg.diff = Int32()
        debug_msg.diff.data = int(self.diff) if self.diff is not None else 0

        self.state_pub.publish(debug_msg)


    def request_new_waypoint(self):
        self.cli = self.create_client(Waypoint, 'mutate_waypoint_queue')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting...')

        # Set up the request with the "get" command
        self.req = Waypoint.Request()
        self.req.command = "get"
        self.req.argument = ""  # No argument needed for the "get" command

        self.future = self.cli.call_async(self.req)
        # Use a callback to handle the response
        self.future.add_done_callback(self.waypoint_response_callback)

    def waypoint_response_callback(self, future: Future):
        try:
            response = future.result()
            if response.success:
                # Parse the received waypoints from the service response
                waypoints = eval(response.message)  # Convert string to list of tuples
                if waypoints:
                    # Extract the next waypoint
                    next_waypoint = waypoints[0]
                    self.get_logger().info(f'New waypoint received: {next_waypoint}')

                    # Convert latitude and longitude to UTM coordinates
                    easting, northing, _, _ = utm.from_latlon(next_waypoint[1], next_waypoint[0])
                    self.curr_dest = Point()
                    self.curr_dest.x = easting
                    self.curr_dest.y = northing
                else:
                    self.get_logger().info('No waypoints available in the queue.')
            else:
                self.get_logger().info('Failed to retrieve waypoints from the service.')
        except Exception as e:
            self.get_logger().error(f'Error in waypoint_response_callback: {str(e)}')


    def curr_gps_callback(self, msg):
        """
        Use the NavSatFix data to assign value to self.curr_loc
        """
        # assuming the zone_number and zone_letter are the same for the current location and the destination
        easting, northing, zone_number, zone_letter = utm.from_latlon(msg.longitude, msg.latitude)
        self.curr_loc = Point()
        self.curr_loc.x = easting
        self.curr_loc.y = northing
        self.zone_number = zone_number
        self.zone_letter = zone_letter

        # Update the distance to destination, check if we have reached our waypoint
        if self.curr_dest is not None:
            dist_to_dest = math.dist((self.curr_loc.x, self.curr_loc.y), (self.curr_dest.x, self.curr_dest.y))
            self.get_logger().info(f'Distance to destination: {dist_to_dest}')
            if dist_to_dest < 5:
                self.request_new_waypoint()

        self.calculate_rudder_angle()

    def wind_callback(self, msg):
        """
        Use the wind data from msg to assign value to self.wind_dir
        """
        self.wind_dir = msg.data

    def heading_dir_callback(self, msg):
        """
        Use the imu data to assign value to self.heading_dir
        """
        data = msg.orientation
        roll_x, roll_y, roll_z = euler_from_quaternion(data.x, data.y, data.z, data.w)
        self.heading_dir = np.degrees(roll_x)

    def calculate_rudder_angle(self):
        """
        The main function for calculating the rudder angle.
        This function will do nothing if current location is missing, or the boat is tacking
        but the tacking point is missing, or the current destination is missing (inclusive or).
        Otherwise, this function will publish the rudder angle, and the rudder angle is 
        from -25 to 25 degree rounded to the nearest 5.

        Precondition: self.curr_dest is not None, self.curr_loc is not None, self.heading_dir is not None
        """
        if self.curr_dest is None or self.curr_loc is None or self.heading_dir is None:
            return # Not enough information to calculate rudder angle yet

        #Handle tacking logic in step
        if self.tacking: 
            final = self.tacking_point
        else:
            final = self.curr_dest
        x_distance = final.x - self.curr_loc.x
        y_distance = final.y - self.curr_loc.y

        target_bearing = np.arctan2(y_distance, x_distance) * 180 / np.pi
        self.get_logger().info(f'Target Bearing: {target_bearing}')

        diff = np.mod(self.heading_dir - target_bearing + 180, 360) - 180
        self.get_logger().info(f'Heading Difference: {diff}')
        self.diff = diff

        rudder_angle = (diff / 180.0) * 25
        self.get_logger().info(f'Rudder Angle Raw: {rudder_angle}')

        # Assuming rudder_angle is a floating-point number and you want it to be an Int32 message
        rudder_angle = np.floor(rudder_angle / 5) * 5  # Floor the angle to the nearest multiple of 5
        rudder_angle = int(rudder_angle)  # Convert to int since Int32 requires an integer value

        self.get_logger().info(f'Rudder Angle: {rudder_angle}')

        # Create an Int32 message and publish the rudder angle
        rudder_angle_msg = Int32()
        rudder_angle_msg.data = rudder_angle

        self.rudder_angle_pub.publish(rudder_angle_msg)
        
    def in_nogo(self):
        """
        Check if the boat is in nogo zone based on the wind direction
        """
        self.get_logger().info(f'Wind Direction: {self.wind_dir}')
        return (150 < self.wind_dir < 210)

    def calculateTP(self):
        """
        Calcualte tacking point to begin tacking. uses winddir + dest
        Assuming that the boat is heading towards the positive x-axis and the destination

        Precondition: self.in_nogo() is true. self.tacking is false. self.curr_loc is not None. self.curr_dest is not None. self.wind_dir is not None.
        """

        assert self.in_nogo(), "Not in nogo zone"
        assert not self.tacking, "Already tacking"
        assert self.curr_loc is not None, "Current location is None"
        assert self.curr_dest is not None, "Current destination is None"
        assert self.wind_dir is not None, "Wind direction is None"

        x = self.curr_loc.x
        y = self.curr_loc.y

        try:
            lat, long = utm.to_latlon(x, y, self.zone_number, self.zone_letter)
            self.get_logger().info(f'Current Location: ({lat}, {long})')
        except Exception as e:
            self.get_logger().error(f'Error in Lat Long: {str(e)}') 

        dist2dest = math.dist((x,y), (self.curr_dest.x,self.curr_dest.y)) 

        if self.wind_dir >= 180 and self.wind_dir <= 210:
            x_TP = x + dist2dest*np.cos(np.deg2rad(45-self.wind_dir))*np.sin(np.deg2rad(45+self.wind_dir))
            y_TP = y - dist2dest*np.cos(np.deg2rad(45-self.wind_dir))*np.cos(np.deg2rad(45+self.wind_dir))
        elif self.wind_dir >= 150 and self.wind_dir <= 180:
            self.wind_dir = 360 - self.wind_dir
            x_TP = x + dist2dest*np.cos(np.deg2rad(45-self.wind_dir))*np.sin(np.deg2rad(45+self.wind_dir))
            y_TP = y + dist2dest*np.cos(np.deg2rad(45-self.wind_dir))*np.cos(np.deg2rad(45+self.wind_dir))
        tp = Point()
        tp.x = x_TP
        tp.y = y_TP

        assert tp.x < 900000 and tp.x > 100000, "Easting out of range"

        # publish new TP
        try:
            lat, long = utm.to_latlon(tp.x, tp.y, self.zone_number, self.zone_letter)
        except Exception as e: 
            self.get_logger().error(f'Tacking point easting: {tp.x}, northing: {tp.y}')
            self.get_logger().error(f'Error in calculateTP: {str(e)}') 
            lat, long = 0., 0.
        nav_sat_msg = NavSatFix()
        nav_sat_msg.longitude = long
        nav_sat_msg.latitude = lat
        nav_sat_msg.position_covariance_type = 0
        self.tacking_point_pub.publish(nav_sat_msg)
        self.get_logger().info('Tacking Point: ' + 'Lat: ' + str(nav_sat_msg.latitude) + ' Long: ' + str(nav_sat_msg.longitude))


        return tp

    def step(self):
        """
        Sail. 
        """
        # FIXME: This stuff starts not as None
        if self.curr_loc is None or (self.tacking and self.tacking_point is None) or self.curr_dest is None:
            # Not enough information to calculate rudder angle yet
            return

        if self.in_nogo() and not self.tacking:
            self.get_logger().info("Beginning Tacking")
            self.tacking = True
            self.tacking_point = self.calculateTP()
            self.tack_time_tracker = 0
        elif self.tacking:
            self.tack_time_tracker += self.timer_period
            if self.tack_time_tracker >= self.tacking_buffer:
                self.get_logger().info("End Tack")

                self.tacking = False
                self.tacking_point = None
        self.calculate_rudder_angle()

        self.get_logger().info("Sailing")


def euler_from_quaternion(x, y, z, w):
        """
        This is a helper function.
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z

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

