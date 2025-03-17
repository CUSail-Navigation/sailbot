import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs import Vector3
from sensor_msgs.msg import NavSatFix
from . import vectornav_real
from . import vectornav_fake

class VectorNav(Node):
    def __init__(self):
        super().__init__('vectornav')

        # Declare timer period parameter
        self.declare_parameter('timer_period', 0.5)
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('vectornav_port', '/dev/ttyUSB0') # FIXME: Add a better default port
        self.vectornav_port = self.get_parameter('vectornav_port').value

        self.declare_parameter('use_fake_data', False)
        self.use_fake_data = self.get_parameter('use_fake_data').value

        self.publisher_gps = self.create_publisher(NavSatFix, '/gps', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu', 10)

        self.timer = self.create_timer(self.timer_period, self.gps_callback)

        if self.use_fake_data:
            self.vectornav = vectornav_fake.FakeVectorNav()
            self.get_logger().info('Using fake data for Vectornav')
        else:
            self.vectornav = vectornav_real.SailVectorNav(self.vectornav_port)
            self.get_logger().info('Launching Vectornav with real data')

    def getYaw(self):
        return self.vectornav.readVectorNavYaw()
    def getPitch(self):
        return self.vectornav.readVectorNavPitch()
    def getRoll(self):
        return self.vectornav.readVectorNavRoll()
    def getLat(self):
        return self.vectornav.readVectorNavLatitude()
    def getLong(self):
        return self.vectornav.readVectorNavLongitude()

    def gps_callback(self):
        nav_sat_msg = NavSatFix()

        nav_sat_msg.longitude = self.getLong()
        nav_sat_msg.latitude = self.getLat()
        nav_sat_msg.position_covariance_type = 0
        self.publisher_gps.publish(nav_sat_msg)

        imu_msg = Vector3()
        yaw = self.getYaw()
        pitch = self.getPitch()
        roll = self.getRoll()

        imu_msg.z = yaw
        imu_msg.y = pitch
        imu_msg.x = roll

        self.publisher_imu.publish(imu_msg)

        self.get_logger().info('Publishing GPS Data: ' + 'Lat: ' + str(nav_sat_msg.latitude) + ' Long: ' + str(nav_sat_msg.longitude))
<<<<<<< HEAD
        self.get_logger().info(f'Publishing IMU Data: Orientation = ({qx}, {qy}, {qz}, {qw})')
        self.get_logger().info(f'Publishing IMU Data: YPR = ({yaw}, {pitch}, {roll})')


    def euler_to_quaternion(self, yaw, pitch, roll):
        """
        Convert Euler angles to quaternion.

        Parameters:
        yaw (float): The yaw angle in radians.
        pitch (float): The pitch angle in radians.
        roll (float): The roll angle in radians.

        Returns:
        list: A list containing the quaternion [qx, qy, qz, qw].
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
=======
        self.get_logger().info(f'Publishing IMU Data (Yaw, Pitch, Roll) = ({imu_msg.z}, {imu_msg.y}, {imu_msg.x})')
>>>>>>> 6c18adab71f0eee88ec192fc708af7d64c4df1ac

def main(args=None):
    rclpy.init(args=args)

    vectornav = VectorNav()

    rclpy.spin(vectornav)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vectornav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

