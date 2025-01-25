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


import math

class MainAlgo(Node):
    """
    The sailing algorithm responsible for changing the rudder angle based on the 
    current location, destination, and heading direction.
    """
    
    def __init__(self):
        super().__init__('main_algo')
        self.get_logger().info('Main-algo started successfully')  # Check if this line prints

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

        # #Subscription for current destination
        # self.subscription_curr_dest = self.create_subscription(
        #     NavSatFix,
        #     '/current_destination',
        #     self.curr_dest_callback,
        #     10)

        # Publisher for rudder angle
        self.rudder_angle_pub = self.create_publisher(Int32, 'algo_rudder', 10)

        # Internal state
        self.curr_loc = Point()
        self.tacking = False
        self.tacking_point = None
        self.heading_dir = 0.0
        self.curr_dest = Point()

        self.request_new_waypoint()

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
                    utm_coords = utm.from_latlon(next_waypoint[1], next_waypoint[0])
                    self.curr_dest.x = utm_coords[0]
                    self.curr_dest.y = utm_coords[1]
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
        utm_coords = utm.from_latlon(msg.longitude, msg.latitude)
        utm_x, utm_y = utm_coords[0], utm_coords[1]
        self.curr_loc = Point()
        self.curr_loc.x = utm_x
        self.curr_loc.y = utm_y

        dist_to_dest = math.dist((self.curr_loc.x,self.curr_loc.y), (self.curr_dest.x,self.curr_dest.y))
        self.get_logger().info(f'Distance to destination: {dist_to_dest}')
        if(dist_to_dest < 5):
            self.request_new_waypoint()
        self.calculate_rudder_angle()


    def heading_dir_callback(self, msg):
        """
        Use the imu data to assign value to self.heading_dir
        """
        data = msg.orientation
        roll_x, roll_y, roll_z = euler_from_quaternion(data.x, data.y, data.z, data.w)
        self.heading_dir = np.degrees(roll_x)
        #self.calculate_rudder_angle()

    # def curr_dest_callback(self, msg):
    #     """
    #     Use the NavSatFix data to assign value to self.curr_dest
    #     """
    #     utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
    #     utm_x, utm_y = utm_coords[0], utm_coords[1]
    #     self.curr_dest = Point()
    #     self.curr_dest.x = utm_x
    #     self.curr_dest.y = utm_y
    #     self.calculate_rudder_angle()

    def calculate_rudder_angle(self):
        """
        The main function for calculating the rudder angle.
        This function will do nothing if current location is missing, or the boat is tacking
        but the tacking point is missing, or the current destination is missing (inclusive or).
        Otherwise, this function will publish the rudder angle, and the rudder angle is 
        from -25 to 25 degree rounded to the nearest 5.
        """
        # x = (5 / 0)
        # self.get_logger().info("Calculating rudder angle.")
        if self.curr_loc is None or (self.tacking and self.tacking_point is None) or self.curr_dest is None:
            # Not enough information to calculate rudder angle yet
            return

        # Choose tacking point or destination based on tacking status
        if self.tacking:
            final = self.tacking_point
            x_distance = final.x - self.curr_loc.x
            y_distance = final.y - self.curr_loc.y
        else:
            final = self.curr_dest
            x_distance = final.x - self.curr_loc.x
            y_distance = final.y - self.curr_loc.y

        target_bearing = np.arctan2(y_distance, x_distance) * 180 / np.pi
        self.get_logger().info(f'Target Bearing: {target_bearing}')

        diff = np.mod(self.heading_dir - target_bearing + 180, 360) - 180
        self.get_logger().info(f'Heading Difference: {diff}')

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

