import math

import rclpy
import utm
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, String


class MuxNode(Node):
    """Multiplexes rudder/sail commands from multiple sources.

    When control_mode is 'algo', the mux auto-selects between 'albo' and
    'rl_algo' depending on whether the current waypoint is upwind.  All other
    modes ('radio', 'webserver', 'controller_app') are passed through directly.

    Upwind detection (same convention as albo):
      - heading:        math angle in degrees (0=East, 90=North)
      - wind angle:     sailbot convention (0=tailwind, 180=headwind)
      - absolute_wind = (wind_angle + heading) % 360  → direction wind blows toward
      - upwind_dir    = (absolute_wind + 180) % 360   → direction wind comes from
      - waypoint bearing computed via atan2(delta_northing, delta_easting) (math angle)
      - waypoint is 'upwind' if |bearing_to_wp - upwind_dir| < upwind_half_angle_deg
    """

    def __init__(self) -> None:
        super().__init__("mux_node")

        self.declare_parameter("control_mode", "algo")
        self.declare_parameter("upwind_half_angle_deg", 60.0)

        self.control_mode = self.get_parameter("control_mode").value
        self.upwind_half_angle_deg = self.get_parameter("upwind_half_angle_deg").value

        self.control_sources = {
            "radio":          {"sail": None, "rudder": None},
            "albo":           {"sail": None, "rudder": None},
            "rl_algo":        {"sail": None, "rudder": None},
            "webserver":      {"sail": None, "rudder": None},
            "controller_app": {"sail": None, "rudder": None},
        }

        for source in self.control_sources:
            self.create_subscription(
                Int32, f"{source}_sail",
                lambda msg, src=source: self._sail_callback(msg, src), 10)
            self.create_subscription(
                Int32, f"{source}_rudder",
                lambda msg, src=source: self._rudder_callback(msg, src), 10)

        self.create_subscription(String, "control_mode", self._control_mode_callback, 10)

        # Sensor subscriptions for upwind detection
        self._wind_angle: float | None = None
        self._heading_deg: float | None = None
        self._boat_lat: float | None = None
        self._boat_lon: float | None = None
        self._wp_lat: float | None = None
        self._wp_lon: float | None = None
        self._active_algo: str = "albo"  # tracks which algo to use when mode=='algo'

        self.create_subscription(Int32, "wind", self._wind_callback, 10)
        self.create_subscription(Vector3, "/imu", self._imu_callback, 10)
        self.create_subscription(NavSatFix, "/gps", self._gps_callback, 10)
        self.create_subscription(NavSatFix, "current_waypoint", self._waypoint_callback, 10)

        self.rudder_pub = self.create_publisher(Int32, "rudder_angle", 10)
        self.sail_pub = self.create_publisher(Int32, "sail", 10)
        self.active_algo_pub = self.create_publisher(String, "active_algo", 10)

        self.create_timer(0.1, self._publish_muxed_values)

    # ── control-source callbacks ──────────────────────────────────────────────

    def _sail_callback(self, msg: Int32, source: str) -> None:
        self.control_sources[source]["sail"] = msg.data

    def _rudder_callback(self, msg: Int32, source: str) -> None:
        self.control_sources[source]["rudder"] = msg.data

    def _control_mode_callback(self, msg: String) -> None:
        if msg.data in self.control_sources or msg.data == "algo":
            self.control_mode = msg.data
            self.get_logger().info(f"Control mode → {msg.data}")
        else:
            self.get_logger().warn(f"Unknown control mode ignored: {msg.data}")

    # ── sensor callbacks ──────────────────────────────────────────────────────

    def _wind_callback(self, msg: Int32) -> None:
        self._wind_angle = float(msg.data)
        self._update_active_algo()

    def _imu_callback(self, msg: Vector3) -> None:
        self._heading_deg = msg.z
        self._update_active_algo()

    def _gps_callback(self, msg: NavSatFix) -> None:
        self._boat_lat = msg.latitude
        self._boat_lon = msg.longitude
        self._update_active_algo()

    def _waypoint_callback(self, msg: NavSatFix) -> None:
        self._wp_lat = msg.latitude
        self._wp_lon = msg.longitude
        self._update_active_algo()

    # ── upwind detection ──────────────────────────────────────────────────────

    def _update_active_algo(self) -> None:
        if None in (self._wind_angle, self._heading_deg, self._boat_lat, self._wp_lat):
            return

        abs_wind = (self._wind_angle + self._heading_deg) % 360
        upwind_dir = (abs_wind + 180) % 360

        boat_e, boat_n, _, _ = utm.from_latlon(self._boat_lat, self._boat_lon)
        wp_e, wp_n, _, _ = utm.from_latlon(self._wp_lat, self._wp_lon)
        wp_bearing = math.degrees(math.atan2(wp_n - boat_n, wp_e - boat_e)) % 360

        diff = ((wp_bearing - upwind_dir + 180) % 360) - 180
        is_upwind = abs(diff) < self.upwind_half_angle_deg

        new_algo = "rl_algo" if is_upwind else "albo"
        if new_algo != self._active_algo:
            self._active_algo = new_algo
            self.get_logger().info(f"Auto-switched algo → {new_algo} (diff={diff:.1f}°)")

    # ── publish ───────────────────────────────────────────────────────────────

    def _publish_muxed_values(self) -> None:
        active = self._active_algo if self.control_mode == "algo" else self.control_mode

        sail_value = self.control_sources[active]["sail"]
        rudder_value = self.control_sources[active]["rudder"]

        if sail_value is not None:
            msg = Int32()
            msg.data = sail_value
            self.sail_pub.publish(msg)

        if rudder_value is not None:
            msg = Int32()
            msg.data = rudder_value
            self.rudder_pub.publish(msg)

        algo_msg = String()
        algo_msg.data = active
        self.active_algo_pub.publish(algo_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
