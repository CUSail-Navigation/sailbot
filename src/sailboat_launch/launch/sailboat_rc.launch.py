import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   """
   This launch file configures soley the remote control handling for the sailboat.
   Control commands can be received from a radio controller or via the
   webserver web app. The mux node is used to multiplex the control commands from the
   radio controller and the webserver app. 
   """
   config = os.path.join(
      get_package_share_directory('sailboat_launch'),
      'config',
      'config.yaml'
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

   # Low-level controller
   ld.add_action(teensy_cmd)

   # RC control
   ld.add_action(radio_cmd)
   ld.add_action(mux_cmd)

   # Telemetry
   ld.add_action(rosbridge_node)

   return ld