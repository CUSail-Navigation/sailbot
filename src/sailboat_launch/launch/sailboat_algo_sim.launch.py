import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   """
   This launch file is used to run the ROS 2 side of the algo simulator. Sensor 
   inputs are received from the simulator via the rosbridge server. The 
   trim_sail and main_algo nodes are run to compute sail and rudder angles. 
   These computed angles are displayed on the simulator web-app.   
   """
   config = os.path.join(
      get_package_share_directory('sailboat_launch'),
      'config',
      'config_sim.yaml'
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


   ld = LaunchDescription()

   # NOTE: no sensor nodes  since the algo simulator simulates the sensor inputs

   # Autonomous sailing
   ld.add_action(main_algo_cmd)
   ld.add_action(trim_sail_cmd)
   ld.add_action(waypoint_service_cmd)

   # The mux is necessary to publish sail and rudder angles to the common 
   # sail_angle and rudder_angle topics. This is necessary if we ever want to
   # run HITL with the simulator.
   ld.add_action(mux_cmd)

   # Simulator communications
   ld.add_action(rosbridge_node)

   return ld