import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from sailboat_interface.srv import Waypoint
from sailboat_interface.msg import StationRectangle
import math
import utm
import time
from threading import Timer

class UTMPoint:
    def __init__(self, easting, northing, zone_number, zone_letter):
        self.easting = easting
        self.northing = northing
        self.zone_number = zone_number
        self.zone_letter = zone_letter

    def to_latlon(self):
        latitude, longitude = utm.to_latlon(
            self.easting, self.northing, self.zone_number, self.zone_letter)
        return LatLongPoint(latitude, longitude)

    def to_navsatfix_msg(self):
        lat_long = self.to_latlon()
        msg = NavSatFix()
        msg.latitude = lat_long.latitude
        msg.longitude = lat_long.longitude
        return msg

    def distance_to(self, other):
        assert self.zone_number == other.zone_number
        return math.dist((self.easting, self.northing), (other.easting, other.northing))

    def __sub__(self, other):
        return UTMPoint(
            self.easting - other.easting,
            self.northing - other.northing,
            self.zone_number,
            self.zone_letter
        )

    def __add__(self, other):
        return UTMPoint(
            self.easting + other.easting,
            self.northing + other.northing,
            self.zone_number,
            self.zone_letter
        )

class LatLongPoint:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def to_utm(self):
        x, y, zone_number, zone_letter = utm.from_latlon(self.latitude, self.longitude)
        return UTMPoint(x, y, zone_number, zone_letter)

class StationKeepingNode(Node):
    def __init__(self):
        super().__init__('station_keeping')

        self.curr_location = None
        self.wind_angle_deg = None
        self.rectangle_corners = []
        self.in_rectangle = False
        self.timer_started = False
        self.current_mode = 'manual'

        self.subscription_curr_loc = self.create_subscription(
            NavSatFix, '/gps', self.curr_gps_callback, 10)
        self.subscription_wind_direction = self.create_subscription(
            Int32, 'wind', self.wind_callback, 10)
        self.mode_sub = self.create_subscription(
            String, 'current_mode', self.mode_callback, 10)
        self.rect_sub = self.create_subscription(
            StationRectangle, 'station_rectangle', self.station_rectangle_callback, 10)

        self.timer_5min = None
        self.timer_30sec = self.create_timer(
            30.0, self.update_perpendicular_waypoints)

    def station_rectangle_callback(self, msg):
        self.rectangle_corners = [LatLongPoint(p.y, p.x).to_utm() for p in msg.corners]
        self.get_logger().info("Received updated station-keeping rectangle corners.")
        center = self.compute_center()
        self.send_waypoints_to_queue([center])

    def mode_callback(self, msg):
        new_mode = msg.data
        if self.current_mode == 'station_keeping' and new_mode != 'station_keeping':
            self.get_logger().info(f"Exiting station keeping mode due to mode change to '{new_mode}'.")
            self.reset_station_keeping()

        self.current_mode = new_mode

        if self.current_mode == 'station_keeping':
            self.get_logger().info("Station keeping mode activated.")

    def reset_station_keeping(self):
        if self.timer_5min:
            self.timer_5min.cancel()
            self.timer_5min = None
            self.get_logger().info("Cancelled 5-minute timer.")

        self.in_rectangle = False
        self.timer_started = False
        self.rectangle_corners = []
        self.get_logger().info("Station keeping state reset.")


    def curr_gps_callback(self, msg):
        self.curr_location = LatLongPoint(
            msg.latitude, msg.longitude).to_utm()
        if self.current_mode == 'station_keeping' and not self.timer_started:
            if self.is_inside_rectangle(self.curr_location):
                self.in_rectangle = True
                self.timer_started = True
                self.get_logger().info("Entered rectangle. Starting 5 min timer.")
                self.timer_5min = Timer(300.0, self.exit_rectangle)
                self.timer_5min.start()

    def wind_callback(self, msg):
        self.wind_angle_deg = msg.data

    def compute_center(self):
        e = sum(p.easting for p in self.rectangle_corners) / 4
        n = sum(p.northing for p in self.rectangle_corners) / 4
        zone = self.rectangle_corners[0]
        return UTMPoint(e, n, zone.zone_number, zone.zone_letter)

    def is_inside_rectangle(self, point):
        xs = [p.easting for p in self.rectangle_corners]
        ys = [p.northing for p in self.rectangle_corners]
        return min(xs) <= point.easting <= max(xs) and min(ys) <= point.northing <= max(ys)

    def update_perpendicular_waypoints(self):
        if self.current_mode != 'station_keeping' or not self.in_rectangle or self.wind_angle_deg is None:
            return

        center = self.compute_center()
        bearing_rad = math.radians((self.wind_angle_deg + 90) % 360)
        dx = 5 * math.cos(bearing_rad)
        dy = 5 * math.sin(bearing_rad)

        wp1 = UTMPoint(
            center.easting + dx, center.northing + dy,
            center.zone_number, center.zone_letter)
        wp2 = UTMPoint(
            center.easting - dx, center.northing - dy,
            center.zone_number, center.zone_letter)

        self.send_waypoints_to_queue([wp1, wp2])

    def send_waypoints_to_queue(self, points):
        self.get_logger().info(f'Sending waypoints to queue: {points}')
        if isinstance(points[0], UTMPoint):
            points = [pt.to_latlon() for pt in points]
        formatted = ';'.join(f'{pt.longitude},{pt.latitude}' for pt in points)

        client = self.create_client(Waypoint, 'mutate_waypoint_queue')
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Waypoint queue service not available')
            return

        req = Waypoint.Request()
        req.command = 'set'
        req.argument = formatted
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: self._handle_wp_response(f))

    def _handle_wp_response(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info(f'Waypoint queue updated: {res.message}')
        else:
            self.get_logger().warn(f'Failed to update queue: {res.message}')

    def exit_rectangle(self):
        self.get_logger().info("5 min timer done. Exiting station keeping.")
        if not self.curr_location:
            return
        best_point = None
        min_dist = float('inf')
        corners = self.rectangle_corners
        for i in range(len(corners)):
            a = corners[i]
            b = corners[(i + 1) % len(corners)]
            proj = self.project_to_segment(self.curr_location, a, b)
            d = proj.distance_to(self.curr_location)
            if d < min_dist:
                min_dist = d
                best_point = proj
        self.send_waypoints_to_queue([best_point])

    def project_to_segment(self, p, a, b):
        ap = p - a
        ab = b - a
        ab_len2 = ab.easting**2 + ab.northing**2
        t = max(0, min(1, (ap.easting * ab.easting + ap.northing * ab.northing) / ab_len2))
        proj_e = a.easting + ab.easting * t
        proj_n = a.northing + ab.northing * t
        return UTMPoint(proj_e, proj_n, a.zone_number, a.zone_letter)


def main(args=None):
    rclpy.init(args=args)
    node = StationKeepingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
