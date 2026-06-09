import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8, Bool
from geometry_msgs.msg import Vector3
from .. import constants


class TrimSail(Node):
    """
    The part of the sailing algorithm that is responsible for calculating
    the optimal angles for the sails based on the wind direction.
    """

    def __init__(self):
        super().__init__('trim_sail')

        # Set to a non-negative value to override the wind sensor with a constant
        # global compass wind direction (degrees, 0-359, where 0/360 = wind from North).
        self.declare_parameter('constant_wind_angle', -1)

        self.heading = None

        # Subscription for wind direction and danger zone.
        self.wind_subscription = self.create_subscription(Int32, 'wind', self.wind_callback, 10)
        self.danger_zone_subscription = self.create_subscription(Bool, 'danger_zone', self.danger_zone_callback, 10)
        self.heading_subscription = self.create_subscription(Vector3, '/imu', self.heading_callback, 10)
        self.in_danger_zone = False

        # Publishers for mainsail angle, jib angle, and jib side flag.
        self.publisher_mainsail_angle = self.create_publisher(Int32, 'algo_sail', 10)
        self.publisher_jib_angle = self.create_publisher(Int32, 'algo_jib_angle', 10)
        self.publisher_jib_side_flag = self.create_publisher(UInt8, 'algo_jib_side_flag', 10)

    def heading_callback(self, msg):
        self.heading = msg.z

    def wind_callback(self, msg):
        """
        Use the wind data from ``msg`` to calculate the sail angles and side;
        publish these values.
        """
        constant_wind = self.get_parameter('constant_wind_angle').value
        if constant_wind >= 0 and self.heading is not None:
            wind_dir = (constant_wind - self.heading) % 360
        else:
            wind_dir = msg.data
        mainsail_angle, jib_angle, jib_side_flag = self.calculate_sail_angles(wind_dir)

        # Create the proper messages and publish the values.
        mainsail_angle_msg = Int32()
        mainsail_angle_msg.data = mainsail_angle
        jib_angle_msg = Int32()
        jib_angle_msg.data = jib_angle
        jib_side_flag_msg = UInt8()
        jib_side_flag_msg.data = jib_side_flag

        self.publisher_mainsail_angle.publish(mainsail_angle_msg)
        self.publisher_jib_angle.publish(jib_angle_msg)
        self.publisher_jib_side_flag.publish(jib_side_flag_msg)
        self.get_logger().info(f"Mainsail angle: {mainsail_angle}, Jib angle: {jib_angle}, Jib side: {jib_side_flag}")

    def calculate_sail_angles(self, wind_dir):
        """
        Calculates the new sail angles given the wind direction.
        Note that goal sail angles are symmetric across the no-go zone.
        We only need to know what side to set the sails on for the jib, since we use 2 servos (2025-2026).
        :param: windDir: the wind direction, an int in [0,360). (CURRENT CONVENTION: 180 = HEAD ON / NO-GO ZONE CENTER).
        :return: a tuple of ints: mainsail_angle, jib_angle, jib_side_flag.
                    mainsail_angle: in [0, 90]. jib_angle: in [10, 80]. jib_side_flag: in {0, 1}.
        """
        half_no_go = constants.WIND.NO_GO_WIDTH / 2

        # Set the jib on the port side of the boat if we're on the left side of the no-go zone.
        jib_side_flag = constants.PHYSICAL.JIB_SIDE_PORT if wind_dir > constants.WIND.NO_GO_CENTER else constants.PHYSICAL.JIB_SIDE_STB

        # No longer care about side: map angles on the left side of the no-go zone symmetrically to the right side.
        if 180 < wind_dir < 360: wind_dir = 360 - wind_dir

        if wind_dir > (constants.WIND.NO_GO_CENTER - half_no_go):  # In the no-go zone.
            mainsail_angle = constants.PHYSICAL.MAINSAIL_MIN_ANGLE
            jib_angle = constants.PHYSICAL.JIB_MIN_ANGLE
        else:  # Not in no-go zone: linearly map wind direction to sail angles.
            mainsail_angle = int(np.interp(wind_dir, [0, constants.WIND.NO_GO_CENTER - half_no_go],
                                    [constants.PHYSICAL.MAINSAIL_MAX_ANGLE, constants.PHYSICAL.MAINSAIL_MIN_ANGLE]))
            jib_angle = int(np.interp(wind_dir, [0, constants.WIND.NO_GO_CENTER - half_no_go],
                                    [constants.PHYSICAL.JIB_MAX_ANGLE, constants.PHYSICAL.JIB_MIN_ANGLE]))

        return mainsail_angle, jib_angle, jib_side_flag

    def danger_zone_callback(self, msg):
        """
        Callback function to handle the danger zone status.
        """
        self.in_danger_zone = msg.data


def main(args=None):
    rclpy.init(args=args)
    trim_sail = TrimSail()

    try:
        rclpy.spin(trim_sail)
    except KeyboardInterrupt:
        pass

    trim_sail.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
