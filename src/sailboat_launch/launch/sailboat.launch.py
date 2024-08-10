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

   return LaunchDescription([
      Node(
            package='sailboat_sensors',
            executable='airmar',
            name='airmar',
            namespace='sailbot',
            parameters=[config]
      ),
      Node(
            package='sailboat_sensors',
            executable='anemometer',
            name='anemometer',
            namespace='sailbot',
            parameters=[config]
      )
   ])