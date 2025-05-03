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
import random

class Particle:
    def __init__(self, x: float, y: float, weight: float = 1.0):
        self.x=x
        self.y=y
        self.weight = weight
        

class ParticleFilter:
    def __init__(self, num_particles: int, search_radius: float, center: UTMPoint):
        """
        Initialize particle filter with circular search area
        :param num_particles: Number of particles to use
        :param search_radius: Radius of search area in meters
        :param center: Center point of search area in UTM coordinates
        """
        self.particles = []
        self.num_particles = num_particles
        self.search_radius = search_radius
        self.center = center
        self.initialize_particles()
    
    def initialize_particles(self):
        """Initialize particles uniformly within circular search area"""
        for _ in range(self.num_particles):
            # Generate random angle and radius
            angle = random.uniform(0, 2 * math.pi)
            radius = math.sqrt(random.uniform(0, 1)) * self.search_radius
            
            # Convert to Cartesian coordinates relative to center
            x = self.center.easting + radius * math.cos(angle)
            y = self.center.northing + radius * math.sin(angle)
            
            self.particles.append(Particle(x, y, 1.0/self.num_particles))
        
    def update_weights(self, bearing, boat_pos, observed_range):
        """
        Update particle weights based on bearing measurement
        :param bearing: Measured bearing to buoy in degrees (0-360)
        :param boat_pos: Current boat position in UTM
        """
        total_weight = 0
        for p in self.particles:
            dx = p.x - boat_pos.easting
            dy = p.y - boat_pos.northing
            predicted_bearing = math.degrees(math.atan2(dx, dy)) % 360
            bearing_error = min(abs(predicted_bearing - bearing), 360 - abs(predicted_bearing - bearing))

            bearing_likelihood = math.exp(-bearing_error ** 2 / (2 * 20 ** 2))  # assume ~20 deg noise

            # range likelihood (assume Gaussian noise)
            predicted_range = math.hypot(dx, dy)

            if(observed_range > 7.5): # assume beyond 7.5 meters we are uncertain, sorry for magic numbers
                # Saturated: range is unreliable
                if predicted_range < 6.5:
                    range_likelihood = 0.01  # penalize particles too close
                else:
                    range_likelihood = 1.0  # neutral

            p.weight = bearing_likelihood * range_likelihood
            total_weight += p.weight
        
        #Normalizing weights
        if total_weight > 0:
            for p in self.particles:
                p.weight /= total_weight
                
    def resample(self):
        """Resample particles based on weights."""
        weights = [p.weight for p in self.particles]
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=weights)
        new_particles = [Particle(self.particles[i].x, self.particles[i].y) for i in indices]
        
        # Add slight random noise to prevent particle deprivation 
        for p in new_particles:
            p.x += random.gauss(0, 2)  
            p.y += random.gauss(0, 2)
        
        self.particles = new_particles
        # Reset weights after resampling
        for p in self.particles:
            p.weight = 1.0 / self.num_particles

    def estimate(self):
        """Estimate buoy position as weighted average of particles"""
        x = sum(p.x * p.weight for p in self.particles)
        y = sum(p.y * p.weight for p in self.particles)
        return x, y
    
     
    def shift_particles(self, dx: float, dy: float):
        """Shift all particles by (dx, dy) to account for boat movement"""
        for p in self.particles:
            p.x += dx
            p.y += dy

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

        # Subscription for buoy position
        self.subscription_buoy_pos = self.create_subscription(
            Point,
            '/buoy_position',
            self.buoy_position_callback,
            10)

        # Internal state
        self.wind_dir = None
        self.curr_loc = None
        self.heading_dir = None

        #Particle filter initizliation
        self.num_particles = 100
        self.search_radius = 100
        self.particle_filter = None
        

        self.waypoints = []

        self.initialize_search_pattern()

        self.get_logger().info('Buoy search algo started successfully')  # Check if this line prints


    #Method to initialize the particle filter
    def initialize_particle_filter(self):
        """Initialize the particle filter with particles over the search area."""
        if self.curr_loc is not None:
            self.particle_filter = ParticleFilter(
                num_particles=self.num_particles,
                search_radius=self.search_radius,
                center=self.curr_loc
            )
            self.get_logger().info('Particle filter initialized.')
        else:
            self.get_logger().error('Cannot initialize particle filter: Current location is None.')


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
        """Callback to handle buoy position updates from the camera."""
        if self.particle_filter is None:
            self.get_logger().info('Buoy detected. Initializing particle filter.')
            self.initialize_particle_filter()

        # Calculate the relative bearing from the camera's frame
        relative_bearing = math.degrees(math.atan2(msg.x, msg.y)) % 360

        observed_range = math.sqrt(msg.x**2 + msg.y**2)

        # Perform one full filter step using the relative bearing
        self.perform_particle_filter_step(relative_bearing, observed_range)
                
    def perform_particle_filter_step(self, relative_bearing: float, observed_range: float):
        """Perform one step of the particle filter process."""
        if self.heading_dir is not None:
            global_bearing = (relative_bearing + self.heading_dir) % 360
        else:
            self.get_logger().error('Heading direction is not available.')
            return

        if self.curr_loc is not None:
            self.particle_filter.update_weights(global_bearing, self.curr_loc, observed_range)
            self.particle_filter.resample()

            estimated_x, estimated_y = self.particle_filter.estimate()
            estimated_buoy = UTMPoint(estimated_x, estimated_y, self.curr_loc.zone_number, self.curr_loc.zone_letter)

            self.get_logger().info(f"Estimated buoy position: {estimated_buoy}")

            # Push the estimated position as a waypoint
            navsat_msg = estimated_buoy.to_navsatfix_msg()
            waypoint_str = f"{navsat_msg.latitude},{navsat_msg.longitude}"
            self.push_waypoint(waypoint_str)

            # Shift particles based on boat movement
            dx = self.curr_loc.easting - estimated_buoy.easting
            dy = self.curr_loc.northing - estimated_buoy.northing
            #self.particle_filter.shift_particles(dx, dy)
        else:
            self.get_logger().error('Current location is not available.')

    
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

