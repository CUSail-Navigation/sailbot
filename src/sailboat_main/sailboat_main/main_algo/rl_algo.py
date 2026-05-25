"""ROS2 node: runs a Stable-Baselines3 PPO model for sailboat waypoint navigation.

Observation vector (13 floats, all clipped to [-1, 1]):
  [0]  rel_wp_x / waypoint_max_radius   -- boat-frame x offset to waypoint
  [1]  rel_wp_y / waypoint_max_radius   -- boat-frame y offset to waypoint
  [2]  distance / waypoint_max_radius   -- scalar distance
  [3]  rel_dir_x                        -- unit vector toward waypoint in boat frame (x)
  [4]  rel_dir_y                        -- unit vector toward waypoint in boat frame (y)
  [5]  u / speed_scale                  -- surge (forward) speed
  [6]  v / speed_scale                  -- sway (sideways) speed
  [7]  r / yaw_rate_scale               -- yaw rate (rad/s)
  [8]  wind_boat_x                      -- apparent wind in boat frame (x = forward)
  [9]  wind_boat_y                      -- apparent wind in boat frame (y = port)
  [10] _WIND_SPEED_OBS                  -- constant (no wind speed sensor)
  [11] last_action[0]                   -- previous rudder command
  [12] last_action[1]                   -- previous sail command

Action vector (2 floats in [-1, 1]):
  [0]  rudder: action * max_rudder_deg degrees
  [1]  sail:   (action + 1) / 2 * max_sail_deg degrees

Coordinate conventions:
  - Boat frame: x = bow, y = port
  - IMU heading: math convention (0 = East, 90 = North) — VectorNav applies (450 - raw) % 360
  - Wind angle:  sailbot convention (0 = tailwind, 180 = headwind)
  - World frame: UTM (x = east, y = north)

Note: stable_baselines3 must be installed in the ROS2 Python environment.
      Run: pip install stable-baselines3
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

import utm

# Wind speed is unavailable; use a fixed normalised value matching training scale (5 m/s / 15 m/s).
_WIND_SPEED_OBS: float = 5.0 / 15.0


class RLAlgo(Node):
    """Sail to the current waypoint using a pre-trained PPO model."""

    current_lat: Optional[float] = None
    current_lon: Optional[float] = None
    waypoint_lat: Optional[float] = None
    waypoint_lon: Optional[float] = None
    heading_deg: Optional[float] = None
    wind_angle: Optional[float] = None

    _prev_utm_x: Optional[float] = None
    _prev_utm_y: Optional[float] = None
    _prev_gps_t: Optional[float] = None
    _vel_east: float = 0.0
    _vel_north: float = 0.0

    _prev_heading: Optional[float] = None
    _prev_heading_t: Optional[float] = None
    _yaw_rate: float = 0.0  # rad/s

    def __init__(self) -> None:
        super().__init__("rl_algo")

        self.declare_parameter(
            "model_path",
            "/home/asun121/sailbench/runs/good_vmg_stable/best_model/best_model.zip",
        )
        self.declare_parameter("timer_period", 0.200)
        self.declare_parameter("max_rudder_deg", 35.0)
        self.declare_parameter("max_sail_deg", 85.0)
        self.declare_parameter("speed_scale", 6.0)
        self.declare_parameter("yaw_rate_scale", 2.0)
        self.declare_parameter("waypoint_max_radius_m", 25.0)
        self.declare_parameter("deterministic", True)

        self.timer_period = self.get_parameter("timer_period").value
        self.max_rudder_deg = self.get_parameter("max_rudder_deg").value
        self.max_sail_deg = self.get_parameter("max_sail_deg").value
        self.speed_scale = self.get_parameter("speed_scale").value
        self.yaw_rate_scale = self.get_parameter("yaw_rate_scale").value
        self.waypoint_max_radius_m = self.get_parameter("waypoint_max_radius_m").value
        self.deterministic = self.get_parameter("deterministic").value

        self._last_action = np.zeros(2, dtype=np.float32)
        self.model = None
        self._load_model(self.get_parameter("model_path").value)

        self.create_subscription(NavSatFix, "/gps", self._gps_callback, 10)
        self.create_subscription(Vector3, "/imu", self._imu_callback, 10)
        self.create_subscription(Int32, "wind", self._wind_callback, 10)
        self.create_subscription(NavSatFix, "current_waypoint", self._waypoint_callback, 10)

        self.rudder_pub = self.create_publisher(Int32, "rl_algo_rudder", 10)
        self.sail_pub = self.create_publisher(Int32, "rl_algo_sail", 10)

        self.create_timer(self.timer_period, self._step)
        self.get_logger().info("RL algo started")

    @staticmethod
    def _cuda_available() -> bool:
        try:
            import torch
            return torch.cuda.is_available()
        except ImportError:
            return False

    def _load_model(self, path: str) -> None:
        try:
            from stable_baselines3 import PPO

            self.model = PPO.load(path, device="cuda" if self._cuda_available() else "cpu")
            self.get_logger().info(f"Running on {'CUDA' if self._cuda_available() else 'CPU'}")
            self.get_logger().info(f"Loaded PPO model from {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

    def _step(self) -> None:
        if self.model is None:
            return
        if None in (self.current_lat, self.waypoint_lat, self.heading_deg):
            return

        obs = self._build_observation()
        action, _ = self.model.predict(obs, deterministic=self.deterministic)
        action = np.asarray(action, dtype=np.float32)
        self._last_action = action

        rudder_deg = float(action[0]) * self.max_rudder_deg
        sail_deg = 0.5 * (float(action[1]) + 1.0) * self.max_sail_deg

        msg = Int32()
        msg.data = int(round(np.clip(rudder_deg, -self.max_rudder_deg, self.max_rudder_deg)))
        self.rudder_pub.publish(msg)

        msg = Int32()
        msg.data = int(round(np.clip(sail_deg, 0.0, self.max_sail_deg)))
        self.sail_pub.publish(msg)

    def _build_observation(self) -> np.ndarray:
        boat_e, boat_n, _, _ = utm.from_latlon(self.current_lat, self.current_lon)
        wp_e, wp_n, _, _ = utm.from_latlon(self.waypoint_lat, self.waypoint_lon)
        dx_east = wp_e - boat_e
        dy_north = wp_n - boat_n
        distance = max(math.hypot(dx_east, dy_north), 1e-9)

        # IMU gives math-convention heading (0=East, 90=North); use directly.
        H_rad = math.radians(self.heading_deg)
        c = math.cos(H_rad)
        s = math.sin(H_rad)

        rel_wp_x = c * dx_east + s * dy_north
        rel_wp_y = -s * dx_east + c * dy_north
        dir_x = rel_wp_x / distance
        dir_y = rel_wp_y / distance

        u = c * self._vel_east + s * self._vel_north
        v = -s * self._vel_east + c * self._vel_north

        # Sailbot wind convention: 0=tailwind, 180=headwind → boat-frame vector [cos, sin].
        if self.wind_angle is not None:
            w_rad = math.radians(self.wind_angle)
            wind_x = math.cos(w_rad)
            wind_y = math.sin(w_rad)
        else:
            wind_x, wind_y = 0.0, 0.0

        return np.array(
            [
                np.clip(rel_wp_x / self.waypoint_max_radius_m, -1.0, 1.0),
                np.clip(rel_wp_y / self.waypoint_max_radius_m, -1.0, 1.0),
                np.clip(distance / self.waypoint_max_radius_m, 0.0, 1.0),
                np.clip(dir_x, -1.0, 1.0),
                np.clip(dir_y, -1.0, 1.0),
                np.clip(u / self.speed_scale, -1.0, 1.0),
                np.clip(v / self.speed_scale, -1.0, 1.0),
                np.clip(self._yaw_rate / self.yaw_rate_scale, -1.0, 1.0),
                np.clip(wind_x, -1.0, 1.0),
                np.clip(wind_y, -1.0, 1.0),
                _WIND_SPEED_OBS,
                float(self._last_action[0]),
                float(self._last_action[1]),
            ],
            dtype=np.float32,
        )

    def _gps_callback(self, msg: NavSatFix) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

        if self._prev_utm_x is not None and self._prev_gps_t is not None:
            dt = now - self._prev_gps_t
            if dt > 0.05:
                alpha = 0.3
                self._vel_east = alpha * (e - self._prev_utm_x) / dt + (1 - alpha) * self._vel_east
                self._vel_north = alpha * (n - self._prev_utm_y) / dt + (1 - alpha) * self._vel_north

        self._prev_utm_x = e
        self._prev_utm_y = n
        self._prev_gps_t = now
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def _imu_callback(self, msg: Vector3) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        heading = msg.z

        if self._prev_heading is not None and self._prev_heading_t is not None:
            dt = now - self._prev_heading_t
            if dt > 0.05:
                dh_deg = ((heading - self._prev_heading + 180.0) % 360.0) - 180.0
                alpha = 0.3
                self._yaw_rate = alpha * math.radians(dh_deg / dt) + (1 - alpha) * self._yaw_rate

        self._prev_heading = heading
        self._prev_heading_t = now
        self.heading_deg = heading

    def _wind_callback(self, msg: Int32) -> None:
        self.wind_angle = float(msg.data)

    def _waypoint_callback(self, msg: NavSatFix) -> None:
        self.waypoint_lat = msg.latitude
        self.waypoint_lon = msg.longitude
        self._last_action = np.zeros(2, dtype=np.float32)
        self.get_logger().info(f"New waypoint: ({msg.latitude:.6f}, {msg.longitude:.6f})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLAlgo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
