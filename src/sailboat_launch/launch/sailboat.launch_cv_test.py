import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('sailboat_launch'),
      'config',
      'config_sim.yaml'
      )
   
   teensy_cmd = Node(
      package='sailboat_main',
      executable='teensy',
      name='teensy',
      namespace='sailbot',
      parameters=[config]
   )

   cv_cmd = Node(
      package='sailboat_vision',
      executable='buoy_detection',
      name='buoy_detection',
      namespace='sailbot',
      parameters=[config]
   )

   ld = LaunchDescription()

   # Sensors
   ld.add_action(teensy_cmd)
   ld.add_action(cv_cmd)
   
   return ld