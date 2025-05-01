#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64  # Assuming buoy angle is a float
import math

class BuoyWaypointPublisher:
    def __init__(self):
        rospy.init_node('buoy_waypoint_publisher', anonymous=True)

        # --- Publishers ---
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, NavSatFix, queue_size=10)

        # --- Subscribers ---
        rospy.Subscriber(self.buoy_angle_topic, Float64, self.buoy_angle_callback)
        rospy.Subscriber(self.current_location_topic, NavSatFix, self.current_location_callback)

        # --- Internal State ---
        self.latest_buoy_angle = None
        self.current_location = None

    def buoy_angle_callback(self, msg):
        """
        Callback function to receive buoy angle.
        :param msg: Float64 message containing the buoy angle in radians.
        """
        self.latest_buoy_angle = msg.data
        self.calculate_and_publish_waypoint()

    def current_location_callback(self, msg):
        """
        Callback function to receive the current GPS location.
        :param msg: NavSatFix message containing the latitude and longitude.
        """
        self.current_location = msg
        self.calculate_and_publish_waypoint()

    def calculate_and_publish_waypoint(self):
        """
        Calculates the new waypoint based on the buoy angle and current location, and publishes it.
        """
        if self.latest_buoy_angle is None or self.current_location is None:
            rospy.loginfo_throttle(60, "Waiting for buoy angle and current location to be available.")
            return

        #Get current location (lat/lon) and buoy angle
        lat = self.current_location.latitude
        lon = self.current_location.longitude
        angle_rad = self.latest_buoy_angle

        #Calculate the new waypoint coordinates
        new_lat, new_lon = self.calculate_new_waypoint(lat, lon, angle_rad, self.waypoint_distance)

        # 3. Create and publish the new waypoint message
        waypoint = NavSatFix()

        self.waypoint_pub.publish(waypoint)
        rospy.loginfo(f"Published new waypoint: Lat={new_lat:.6f}, Lon={new_lon:.6f}")

    def calculate_new_waypoint(self, lat_deg, lon_deg, angle_rad, distance_m):
        """
        Calculates a new waypoint (lat/lon) given a starting point, angle, and distance.
        Uses the Haversine formula for more accurate distance calculations on a sphere.
        """
        R = 6371e3  # Radius of Earth in meters

        lat_rad = math.radians(lat_deg)
        lon_rad = math.radians(lon_deg)

        new_lat_rad = 0
                               
        new_lon_rad = 0

        new_lat_deg = math.degrees(new_lat_rad)
        new_lon_deg = math.degrees(new_lon_rad)

        return new_lat_deg, new_lon_deg

if __name__ == '__main__':
    try:
        buoy_waypoint_publisher = BuoyWaypointPublisher()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
