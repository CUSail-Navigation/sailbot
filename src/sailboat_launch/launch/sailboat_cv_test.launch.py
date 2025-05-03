import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   """
   Buoy search.
   """
   config = os.path.join(
      get_package_share_directory('sailboat_launch'),
      'config',
      'config.yaml'
      )

   cv_cmd = Node(
      package='sailboat_vision',
      executable='buoy_detection',
      name='buoy_detection',
      namespace='sailbot',
      parameters=[config]
   )
   
   ld = LaunchDescription()

   # HW peripherals
   ld.add_action(gps_cmd)
   ld.add_action(teensy_cmd)

   # Autonomous sailing
   ld.add_action(main_algo_cmd)
   ld.add_action(trim_sail_cmd)
   ld.add_action(waypoint_service_cmd)

   # Togglable rc control
   ld.add_action(radio_cmd)
   ld.add_action(mux_cmd)

   # Telemetry
   ld.add_action(rosbridge_node)

   return ld
