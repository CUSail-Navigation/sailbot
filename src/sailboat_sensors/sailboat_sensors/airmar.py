import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from . import sail_airmar


class AirMar(Node):
    def __init__(self):
        super().__init__('airmar')

        # Declare timer period parameter
        self.declare_parameter('timer_period', 0.5)
        self.timer_period = self.get_parameter('timer_period').value

        self.declare_parameter('airmar_port')
        self.airmar_port = self.get_parameter('airmar_port').value
        self.publisher_gps = self.create_publisher(NavSatFix, '/gps', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu', 10)

        self.timer = self.create_timer(self.timer_period, self.gps_callback)

        self.airmar = sail_airmar.SailAirMar(self.airmar_port)
        self.get_logger().info('Launching Airmar')

    def getHeading(self):
        return self.airmar.readAirMarHeading()
    def getLat(self):
        return self.airmar.readAirMarLatitude()
    def getLong(self):
        return self.airmar.readAirMarLongitude()

    def gps_callback(self):
        msg = NavSatFix()
        msg.longitude = self.getLong()
        msg.latitude = self.getLat()
        msg.position_covariance_type = 0
        self.publisher_gps.publish(msg)

        msg = Imu()
        yaw = self.getHeading()
        msg.orientation = self.euler_to_quaternion(yaw, 0, 0)
        msg.angular_velocity.z = self.airmar.readAirMarROT()
        self.publisher_imu.publish(msg)

        self.get_logger().info('Publishing Airmar Data: ' + 'Lat: ' + str(msg.latitude) + ' Long: ' + str(msg.longitude) + ' Heading: ' + str(yaw))


    def euler_to_quaternion(self, yaw, pitch, roll):

            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)

    airmar = AirMar()

    rclpy.spin(airmar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    airmar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

