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


   mux_cmd = Node(
      package='sailboat_main',
      executable='mux',
      name='mux',
      namespace='sailbot',
      parameters=[config]
   ) 

   main_algo_cmd  = Node(
      package='sailboat_main',
      executable='main_algo',
      name='main_algo',
      namespace='sailbot',
      parameters=[config]
   )

   teensy_cmd = Node(
    package='sailboat_main',
    executable='teensy',
    name='teensy',
    namespace='sailbot',
    parameters=[config]
   )

   trim_sail_cmd = Node(
      package='sailboat_main',
      executable='trim_sail',
      name='trim_sail',
      namespace='sailbot',
      parameters=[config]
   )

   waypoint_service_cmd = Node(
      package='sailbot_events',
      executable='waypoint_service',
      name='waypoint_service',
      namespace='sailbot',
      parameters=[config]
   )



   ld = LaunchDescription()

   # Sensors
   ld.add_action(gps_cmd)
   ld.add_action(main_algo_cmd)
   ld.add_action(trim_sail_cmd)
   ld.add_action(waypoint_service_cmd)
   ld.add_action(mux_cmd)
   ld.add_action(teensy_cmd)

   return ld