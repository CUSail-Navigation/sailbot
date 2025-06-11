import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   """
   This launch file is the nominal case for the sailboat. It requires real hardware, 
   and exposes a rosbridge websocket for telemetry. The control mode can be toggled
   between autonomous and remote control. The mux node is used to multiplex the 
   control mode and the control sources.
   """
   config = os.path.join(
      get_package_share_directory('sailboat_launch'),
      'config',
      'config.yaml'
      )
      
   gps_cmd = Node(
      package='sailboat_sensors',
      executable='vectornav',
      name='vectornav',
      namespace='sailbot',
      parameters=[{'use_fake_data': True}]
   )

   teensy_cmd = Node(
         package='sailboat_main',
         executable='teensy',
         name='teensy',
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
      executable='algo',
      name='algo',
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

   rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090  # This is the default WebSocket port for rosbridge
        }]
    )

   radio_cmd = Node(
      package='sailboat_main',
      executable='radio',
      name='radio',
      namespace='sailbot',
      parameters=[config]
   )

   camera_node = Node(
      package='sailboat_vision',
      executable='buoy_detection',
      name='buoy_detection',
      namespace='sailbot',
      parameters=[config]
   )


   search_node = Node(
      package='sailboat_vision',
      executable='buoy_search',
      name='buoy_search',
      namespace='sailbot',
      parameters=[config]
   )
   
   
   mode_manager_cmd = Node(
      package='sailbot_events',
      executable='mode_manager',
      name='mode_manager',
      namespace='sailbot',
      parameters=[config]
   )
   
   ld = LaunchDescription()
   
   #Event drivers
   ld.add_action(mode_manager_cmd)

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

   # Computer Vision
   ld.add_action(camera_node)
   ld.add_action(search_node)

   return ld