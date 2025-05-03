def generate_launch_description():
   """
   Launch file for CV
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
      parameters=[config]
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

   cv_cmd = Node(
      package='sailboat_vision',
      executable='buoy_detection',
      name='buoy_detection',
      namespace='sailbot',
      parameters=[config]
   )

   buoy_search_cmd = Node(
      package='sailboat_vision',
      executable='buoy_search',
      name='buoy_search',
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

   ld.add_action(cv_cmd)
   ld.add_action(buoy_search_cmd)

   return ld
