import matplotlib.pyplot as plt
import numpy as np
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Int32, Float32
from rclpy.task import Future
from typing import Optional

from sailboat_interface.srv import Waypoint
from sailboat_interface.msg import AlgoDebug

import utm
import math
import random


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
        return f"UTMPoint(x={self.easting}, y={self.northing}, zone_number={self.zone_number}, zone_letter={self.zone_letter})"

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
        :param bearing: Measured bearing of buoy in degrees relative to east (0-360)
        :param boat_pos: Current boat position in UTM
        """
        total_weight = 0
        for p in self.particles:
            dx = p.x - boat_pos.easting
            dy = p.y - boat_pos.northing
            predicted_bearing = np.arctan2(dy, dx) * 180 / np.pi

            bearing_error = (predicted_bearing - bearing) % 360
            bearing_likelihood = math.exp(-bearing_error ** 2 / (2 * 20 ** 2))  # assume ~20 deg noise

            # range likelihood (assume Gaussian noise)
            predicted_range = math.hypot(dx, dy)
            range_error = abs(predicted_range - observed_range)

            if(observed_range > 15): # assume beyond 15 meters we are uncertain, sorry for magic numbers
                range_likelihood = math.exp(-range_error ** 2 / (2 * 4 ** 2))  # assume ~4m stddev
            else:
                range_likelihood = math.exp(-range_error ** 2 / (2 * 2 ** 2)) # assume ~2m stddev
            p.weight = bearing_likelihood * range_likelihood
            total_weight += p.weight
        
        # Normalizing weights
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
            'buoy_position',
            self.buoy_position_callback,
            10)
        
        # Subscription for mode
        self.current_mode = 'manual'  # Default mode
        self.mode_sub = self.create_subscription(
            String,
            'current_mode',
            self.mode_callback,
            10)

        self.estimate_distance_pub = self.create_publisher(
            Float32,
            'buoy_distance',
            10
        )

        self.control_mode_pub = self.create_publisher(
            String,
            'control_mode',
            10
        )


        # Publishers to finish at the buoy
        self.rudder_pub = self.create_publisher(Int32, 'rudder_angle', 10)
        self.sail_pub = self.create_publisher(Int32, 'sail', 10)

        # Internal state
        self.wind_dir = None
        self.curr_loc = None
        self.heading_dir = None

        self.center = None
        self.search_direction = None

        #Particle filter initizliation
        self.num_particles = 100
        self.search_radius = 100
        self.particle_filter = None

        self.waypoints = []

        #Initiialize debug plot
        self.show_debug_plot = False
        if self.show_debug_plot:
            self.setup_debug_plot()
        
        self.get_logger().info('Buoy search algo started successfully')

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
        if self.current_mode != 'search':
            return
        if self.particle_filter is None:
            self.get_logger().info('Buoy detected. Initializing particle filter.')
            self.initialize_particle_filter()

        # Calculate the relative bearing from the camera's frame
        relative_bearing = math.degrees(math.atan2(msg.x, msg.y)) % 360

        observed_range = math.sqrt(msg.x**2 + msg.y**2)

        # Perform one full filter step using the relative bearing
        self.perform_particle_filter_step(relative_bearing, observed_range)
        
    def mode_callback(self, msg) :
        new_mode = msg.data
        if self.current_mode == 'search' and new_mode != 'search':
            self.get_logger().info(f"[Buoy search node] Exiting search mode due to mode change to '{new_mode}'.")
            self.reset_search()

        self.current_mode = new_mode

        if self.current_mode == 'search':
            self.get_logger().info("[Buoy search node] Search mode activated.")
            self.initialize_search_pattern()  # Ensure search pattern is initialized on entering search mode
    
    def reset_search(self):
        """
        Reset the search state, including the particle filter and waypoints.
        """
        self.get_logger().info("Resetting search state.")
        self.particle_filter = None
        self.waypoints = []
        self.center = None
        self.search_direction = None
            
    def set_waypoints(self, waypoints):
        """
        Send a request to the webserver to set its waypoint queue.
        """
        if self.current_mode != 'search':
            return
        self.cli = self.create_client(Waypoint, 'mutate_waypoint_queue')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting...')

        self.waypoints = waypoints

        if self.show_debug_plot:
            self.update_debug_plot()

        # Set up the request with the "set" command
        self.req = Waypoint.Request()
        self.req.command = "set"
        self.req.argument = ';'.join([f"{x},{y}" for x, y in self.waypoints])

        self.future = self.cli.call_async(self.req)
        # Use a callback to handle the response
        self.future.add_done_callback(self.set_response_callback)

    def set_response_callback(self, future: Future):
        """
        Log set request response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Waypoints set successfully.')
            else:
                self.get_logger().info('Failed to set waypoint from the service.')
        except Exception as e:
            self.get_logger().error(f'Error in waypoint_response_callback: {str(e)}')
                
    def perform_particle_filter_step(self, relative_bearing: float, observed_range: float):
        """Perform one step of the particle filter process."""
        if self.current_mode != 'search':
            return
        if self.heading_dir is not None:
            global_bearing = (self.heading_dir - relative_bearing) % 360
        else:
            self.get_logger().error('Heading direction is not available.')
            return

        if self.curr_loc is not None and self.heading_dir is not None:
            self.particle_filter.update_weights(global_bearing, self.curr_loc, observed_range)
            self.particle_filter.resample()
            if self.show_debug_plot:
                self.update_debug_plot()

            estimated_x, estimated_y = self.particle_filter.estimate()
            try:
                estimated_buoy = UTMPoint(estimated_x, estimated_y, self.curr_loc.zone_number, self.curr_loc.zone_letter)
                estim_distance = estimated_buoy.distance_to(self.curr_loc)
                estim_distance_msg = Float32()
                estim_distance_msg.data = estim_distance
                self.estimate_distance_pub.publish(estim_distance_msg)
                if abs(estim_distance) < 3:
                    self.get_logger().info("BANG BANG")
                    # self.current_mode = 'manual'
                    mode_msg = String()
                    mode_msg.data = "controller_app"
                    self.control_mode_pub.publish(mode_msg)
                    
                    sail_out_msg = Int32()
                    tail_neutral_msg = Int32()

                    sail_out_msg.data = 90
                    tail_neutral_msg.data = 0

                    self.rudder_pub.publish(tail_neutral_msg)
                    self.sail_pub.publish(sail_out_msg)

            except utm.error.OutOfRangeError: # Catches when between different UTM zones
                self.get_logger().warn(f"Invalid UTM")
                return

            self.get_logger().info(f"Estimated buoy position: {estimated_buoy}")

            

            # Push the estimated position as a waypoint
            estimated_buoy_latlon = estimated_buoy.to_latlon()
            self.set_waypoints([[estimated_buoy_latlon.latitude, estimated_buoy_latlon.longitude]])
        else:
            self.get_logger().error('Current location is not available.')

    def setup_debug_plot(self):
        """Initialize the matplotlib plot for real-time debugging."""
        try:
            self.fig, self.ax = plt.subplots(figsize=(8, 6))
            plt.ion()  # Enable interactive mode
            self.fig.show()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize debug plot: {str(e)}')
            self.fig = None
            self.ax = None
    
    def update_debug_plot(self):
        """Update the matplotlib plot with current state."""
        if self.fig is None or self.ax is None:
            return

        self.ax.clear()
        
        # Plot particles
        if self.particle_filter is not None:
            xs = [p.x for p in self.particle_filter.particles]
            ys = [p.y for p in self.particle_filter.particles]
            self.ax.scatter(xs, ys, c='r', s=10, label='Particles', alpha=0.5)
            
        # Plot waypoints
        if self.waypoints:
            utm_points = [LatLongPoint(lat, lon).to_utm() for lat, lon in self.waypoints]
            wx = [p.easting for p in utm_points]
            wy = [p.northing for p in utm_points]
            self.ax.plot(wx, wy, 'bo-', label='Waypoints')
            
        # Plot current location
        if self.curr_loc is not None:
            self.ax.scatter([self.curr_loc.easting], [self.curr_loc.northing],
                           c='g', s=50, marker='*', label='Boat')
            
        self.ax.set_title('Particle Filter & Search Pattern')
        self.ax.set_xlabel('Easting (m)')
        self.ax.set_ylabel('Northing (m)')
        self.ax.legend()
        self.ax.grid(True)
        
        try:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001) 
        except Exception as e:
            self.get_logger().error(f'Failed to update or save plot: {str(e)}')


    def initialize_particle_filter(self):
        """Initialize the particle filter with particles over the search area."""
        if self.current_mode != 'search':
            return
        if self.curr_loc is not None:
            self.particle_filter = ParticleFilter(
                num_particles=self.num_particles,
                search_radius=self.search_radius,
                center=self.curr_loc
            )
            self.get_logger().info('Particle filter initialized.')
            if self.show_debug_plot:
                self.update_debug_plot()
        else:
            self.get_logger().error('Cannot initialize particle filter: Current location is None.')

    def initialize_search_pattern(self):
        """
        Initialize a search pattern that zigzags across a circle.
        """
        if self.current_mode != 'search':
            return
        expansion_step = 45
        max_radius = 100
        direction = 1

        center = LatLongPoint(42.27538, -71.75590).to_utm()
        self.search_direction = self.search_direction if self.search_direction is not None else 0

        search_pattern = []
        x = -1 * max_radius
        while x <= max_radius:
            y = direction * math.sqrt(max_radius ** 2 - x ** 2)
            search_pattern.append([x, y])
            direction *= -1
            x += expansion_step
        
        search_pattern_waypoints = []
        rotation_angle = math.radians(self.search_direction)
        easting_translation = center.easting
        northing_translation = center.northing

        for x, y in search_pattern:
            easting = x * math.cos(rotation_angle) - y * math.sin(rotation_angle)
            northing = x * math.sin(rotation_angle) + y * math.cos(rotation_angle)
            easting += easting_translation
            northing += northing_translation

            try:
                new_waypoint = UTMPoint(easting, northing, center.zone_number, center.zone_letter).to_latlon()
            except utm.error.OutOfRangeError: # Catches when between different UTM zones
                self.get_logger().warn(f"Invalid UTM")
                return
    
            search_pattern_waypoints.append((new_waypoint.latitude, new_waypoint.longitude))
        self.set_waypoints(search_pattern_waypoints)

        if self.show_debug_plot:
            self.update_debug_plot()

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

