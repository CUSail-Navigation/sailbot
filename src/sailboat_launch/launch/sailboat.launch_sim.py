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

   main_algo_cmd  = Node(
      package='sailboat_main',
      executable='main_algo',
      name='main_algo',
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

   event_driver_cmd  = Node(
      package='sailbot_events',
      executable='event_driver',
      name='event_driver',
      namespace='sailbot',
      parameters=[config]
   )

   ld = LaunchDescription()

   # Sensors
   ld.add_action(gps_cmd)
   ld.add_action(anemometer_cmd)
   ld.add_action(servo_cmd)
   ld.add_action(main_algo_cmd)
   ld.add_action(trim_sail_cmd)
   ld.add_action(event_driver_cmd)

   return ld