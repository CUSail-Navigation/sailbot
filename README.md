# sailros2
Unfinished implementation of raspberrypi for CUSail running ROS2 Humble: [Download ROS2 Humble](https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html)

## Building the sailros2 package
Before running any nodes, first build the sailros2 project with the following steps:
  Be sure to first source the ```setup.bat``` file by typing: ```\opt\ros\humble\x64\setup.bat```
  Got to the directory of the sailros2 project: ```FILE_PATH\sailros2```
  Type ```colcon build``` to build your ros2 project
      Note that to build a specific package, we can run
      ```colcon build --merge-install --packages-select <package_name>```
  
  Once the package is built an install folder should be created. Launch the local ros2 environment by doing the following:
  Go to the ```\sailros2\install``` directory then type ```local_setup.bat``` to source the local environment

## Running nodes
If the package is built, you can now directly run any nodes in the package. We can run the anemometer test node with: ```ros2 run sailboat_sensors test_read_wind```

Note: Running nodes in ros2 is in the form - ```ros2 run <package_name> <executable_name>```
