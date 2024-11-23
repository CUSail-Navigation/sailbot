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

   teensy_cmd = Node(
      package='sailboat_main',
      executable='teensy',
      name='teensy',
      namespace='sailbot',
      parameters=[config]
   )

   radio_cmd = Node(
      package='sailboat_main',
      executable='radio',
      name='radio',
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

   trim_sail_cmd = Node(
      package='sailboat_main',
      executable='trim_sail',
      name='trim_sail',
      namespace='sailbot',
      parameters=[config]
   )

   # event_driver_cmd  = Node(
   #    package='sailbot_events',
   #    executable='event_driver',
   #    name='event_driver',
   #    namespace='sailbot',
   #    parameters=[config]
   # )

   waypoint_service_cmd = Node(
      package='sailbot_events',
      executable='waypoint_service',
      name='waypoint_service',
      namespace='sailbot',
      parameters=[config]
   )

   # Add the rosbridge_server node
   rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090  # This is the default WebSocket port for rosbridge
        }]
    )


   ld = LaunchDescription()

   # Sensors
   ld.add_action(gps_cmd)
   ld.add_action(teensy_cmd)
   ld.add_action(main_algo_cmd)
   ld.add_action(trim_sail_cmd)
   #ld.add_action(event_driver_cmd)
   ld.add_action(waypoint_service_cmd)
   ld.add_action(radio_cmd)
   ld.add_action(mux_cmd)
   ld.add_action(rosbridge_node)

   return ld