import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, String
from typing import Optional
import utm


# ---------------------------------------------------------------------------
# Coordinate helpers (same conventions as algo.py)
# ---------------------------------------------------------------------------

class UTMPoint:
    def __init__(self, easting, northing, zone_number, zone_letter):
        self.easting = easting
        self.northing = northing
        self.zone_number = zone_number
        self.zone_letter = zone_letter

    def distance_to(self, other: 'UTMPoint') -> float:
        return math.dist((self.easting, self.northing), (other.easting, other.northing))

    def bearing_to(self, other: 'UTMPoint') -> float:
        """Compass bearing from self to other, degrees [0, 360)."""
        de = other.easting - self.easting
        dn = other.northing - self.northing
        return math.degrees(math.atan2(dn, de)) % 360


class LatLongPoint:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def to_utm(self) -> UTMPoint:
        e, n, zone_num, zone_let = utm.from_latlon(self.latitude, self.longitude)
        return UTMPoint(e, n, zone_num, zone_let)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def signed_angle_diff(a: float, b: float) -> float:
    """Signed difference (a - b) wrapped to [-180, 180]."""
    return float(np.mod(a - b + 180, 360) - 180)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class VMGUpwind(Node):
    """
    Two-mode sailing algorithm:

    UPWIND  — waypoint is inside the no-go zone.
               Tack between port and starboard at best_vmg_twa_deg off the wind.
               Two guards prevent tack chattering:
                 1. layline_hysteresis_deg: must cross the layline by this margin first.
                 2. min_tack_time_s: minimum seconds between tacks.

    DEFAULT — everything else (reach or downwind).
               Steer directly at the waypoint.

    Wind convention (inherited from algo.py):
      wind_direction  — relative to boat, degrees, 180 = headwind
      absolute_wind_dir = wind_direction + heading  — compass bearing wind blows TO
      upwind_dir = absolute_wind_dir + 180          — compass bearing wind comes FROM
    """

    TACK_PORT = 1
    TACK_STBD = -1

    def __init__(self):
        super().__init__('vmg_upwind')

        self.declare_parameter('timer_period',           0.2)
        self.declare_parameter('best_vmg_twa_deg',      45.0)   # tune from sailbench polar
        self.declare_parameter('no_go_zone_deg',        45.0)
        self.declare_parameter('layline_hysteresis_deg', 8.0)
        self.declare_parameter('min_tack_time_s',       15.0)
        self.declare_parameter('kp',                     0.5)
        self.declare_parameter('max_rudder_deg',         25)
        self.declare_parameter('roll_rudder_gain',       0.0)  # deg rudder per deg heel

        self.timer_period  = self.get_parameter('timer_period').value
        self.best_vmg_twa  = self.get_parameter('best_vmg_twa_deg').value
        self.no_go_zone    = self.get_parameter('no_go_zone_deg').value
        self.hysteresis    = self.get_parameter('layline_hysteresis_deg').value
        self.min_tack_time = self.get_parameter('min_tack_time_s').value
        self.kp            = self.get_parameter('kp').value
        self.max_rudder    = int(self.get_parameter('max_rudder_deg').value)
        self.roll_rudder_gain = self.get_parameter('roll_rudder_gain').value

        # Sensor state
        self.heading:       Optional[float]    = None  # compass degrees
        self.absolute_wind: Optional[float]    = None  # compass bearing wind blows TO
        self.location:      Optional[UTMPoint] = None
        self.waypoint:      Optional[UTMPoint] = None
        self.roll:          float              = 0.0   # degrees, positive = starboard up

        # Tack state
        self.tack            = self.TACK_PORT
        self.last_tack_time  = 0.0

        self.create_subscription(NavSatFix, '/gps',              self._on_gps,      10)
        self.create_subscription(Vector3,   '/imu',              self._on_imu,      10)
        self.create_subscription(Int32,     'wind',              self._on_wind,     10)
        self.create_subscription(NavSatFix, 'current_waypoint',  self._on_waypoint, 10)
        self.create_subscription(String,    'current_mode',      self._on_mode,     10)

        self.rudder_pub = self.create_publisher(Int32, 'algo_rudder', 10)
        self.create_timer(self.timer_period, self._step)
        self.get_logger().info('VMG upwind node started')

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def _step(self):
        if any(v is None for v in [self.location, self.waypoint,
                                    self.heading, self.absolute_wind]):
            return

        upwind_dir  = (self.absolute_wind + 180) % 360
        wp_bearing  = self.location.bearing_to(self.waypoint)
        wp_angle    = signed_angle_diff(wp_bearing, upwind_dir)  # + = right of upwind

        if abs(wp_angle) < self.no_go_zone:
            # Upwind mode — tack at best VMG angle
            self._update_tack(wp_angle)
            target = (upwind_dir + self.tack * self.best_vmg_twa) % 360
            mode   = f'upwind/{"port" if self.tack == self.TACK_PORT else "stbd"}'
        else:
            # Reach or downwind — steer straight at waypoint
            target = wp_bearing
            mode   = 'reach/downwind'

        heading_err  = signed_angle_diff(self.heading, target)
        roll_offset  = self.roll_rudder_gain * self.roll
        rudder       = int(np.clip(self.kp * heading_err + roll_offset, -self.max_rudder, self.max_rudder))

        msg = Int32()
        msg.data = rudder
        self.rudder_pub.publish(msg)
        self.get_logger().info(
            f'[{mode}] target={target:.1f} heading={self.heading:.1f} '
            f'err={heading_err:.1f} roll={self.roll:.1f} roll_offset={roll_offset:.1f} rudder={rudder}'
        )

    # ------------------------------------------------------------------
    # Tack management
    # ------------------------------------------------------------------

    def _update_tack(self, wp_angle: float):
        """Switch tacks when the layline is crossed (with hysteresis + time guard)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_tack_time < self.min_tack_time:
            return

        # wp_angle > 0: waypoint right of upwind → natural tack is PORT (+1)
        # Tack when waypoint has crossed to the opposite side by > hysteresis degrees.
        # TACK_PORT (+1): fires when wp_angle < -hysteresis
        # TACK_STBD (-1): fires when wp_angle > +hysteresis
        if wp_angle * self.tack < -self.hysteresis:
            self.tack = -self.tack
            self.last_tack_time = now
            side = 'port' if self.tack == self.TACK_PORT else 'stbd'
            self.get_logger().info(f'Tacking → {side} (wp_angle={wp_angle:.1f})')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_gps(self, msg: NavSatFix):
        self.location = LatLongPoint(msg.latitude, msg.longitude).to_utm()

    def _on_imu(self, msg: Vector3):
        self.heading = msg.z
        self.roll    = msg.x

    def _on_wind(self, msg: Int32):
        if self.heading is not None:
            self.absolute_wind = (float(msg.data) + self.heading) % 360

    def _on_waypoint(self, msg: NavSatFix):
        self.waypoint = LatLongPoint(msg.latitude, msg.longitude).to_utm()

        # Initialise tack toward the waypoint immediately if we have wind data
        if self.absolute_wind is not None and self.location is not None:
            upwind_dir = (self.absolute_wind + 180) % 360
            wp_angle   = signed_angle_diff(self.location.bearing_to(self.waypoint), upwind_dir)
            self.tack  = self.TACK_PORT if wp_angle >= 0 else self.TACK_STBD

        self.last_tack_time = 0.0
        self.get_logger().info(f'New waypoint ({msg.latitude:.6f}, {msg.longitude:.6f})')

    def _on_mode(self, msg: String):
        pass  # available for future mode-gating if needed


def main(args=None):
    rclpy.init(args=args)
    node = VMGUpwind()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
