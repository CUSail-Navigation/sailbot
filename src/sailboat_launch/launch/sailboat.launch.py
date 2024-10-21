import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('sailboat_launch'),
      'config',
      'config.yaml'
      )
   gps_cmd = Node(
      package='sailboat_sensors',
      executable='airmar',
      name='airmar',
      namespace='sailbot',
      parameters=[config]
   )

   anemometer_cmd = Node(
      package='sailboat_sensors',
      executable='anemometer',
      name='anemometer',
      namespace='sailbot',
      parameters=[config]
   )

   servo_cmd = Node(
      package='sailboat_main',
      executable='servo',
      name='servo',
      namespace='sailbot',
      parameters=[config]
   )

   ld = LaunchDescription()

   # Sensors
   ld.add_action(gps_cmd)
   ld.add_action(anemometer_cmd)
   ld.add_action(servo_cmd)

   return ld