# Overview

The teensy directory includes the code for our Teensy 4.0 microcontroller, which serves as the low-level controller of our sailboat. The code manages the sensors, actuators (such as the servos for the sail and rudder), and communicates via the
MiniPC via a serial connection.

## Code Overview
This code is structured based on [Lodestar](https://github.com/shihaocao/lodestar), a small scale electric demonstrator for the belly-flop and tail-sitting control algorithms necessary for SpaceX's Starship.

### main.cpp
This file is comparable to a .ino file you would see in the Arduino IDE (notice setup and loop are exactly the same as they would be in an Arduino file)

### MainControlLoop.cpp
The MainControlLoop initializes and executes all monitors and control tasks.

### SFR
SFR stands for State Field Registry. It contains values that should be available 
to the entire boat. Ie. sensor values, serial buffer data, etc.

### Monitors
Monitors read input from some source and updates sensor values in the SFR.

### Control Tasks
Control tasks perform actions based on the current state of the boat or SFR values.

### constants.hpp
Constants contains values that will never be changed. This prevents "magic numbers".

## Getting Started
Below are steps to set up your development environment to upload code and observe serial outputs from the Teensy.
### Prerequesites: 
- [VSCode](https://code.visualstudio.com/download) is installed
- The [sailbot](https://github.com/CUSail-Navigation/sailbot) repository is cloned
### Steps:
1. In VSCode, click on "Extensions" on the left hand toolbar. Search for an install PlatformIO IDE.
2. Open the "teensy" folder within the sailbot repository in VSCode.
3. Now at the bottom of your screen in the blue toolbar, you should see a check, arrow, and serial monitor icon. If you do not see the blue toolbar, make sure you 
have the "teensy" folder open as the root.
4. If you would just like to compile code but not upload to the teensy, press the check.
5. If you would like to upload to the teensy, press the arrow.
6. To vie wthe serial monitor press the electrical cord icon.

## Developing with a Teensy w/ Docker & WSL on Windows:
The following steps exposes a Windows COM port to WSL and then exposes the WSL port to the Docker image running in WSL. This was necessary to setup a test environment with the ROS2 codebase on my Windows 11 computer.
### Changing Docker Desktop Backend to support WSL
Make sure you have:
- [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/) installed
- [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) installed

1. Open Docker Desktop
2. Navigate to Settings->General
3. Check the box "Use the WSL 2 based engine"
### Expose Windows COM port to WSL 
1. Download and install [USBIPD-WIN](https://github.com/dorssel/usbipd-win/) (and follow their README.md instructions)
2. Open PowerShell as an administrator.
3. Obtain a list of USB devices using ```usbipd list```.
4. Find the bus ID of the device (e.g. 4-4) and use ```usbipd bind --busid <id>``` to share it with WSL.
5. Use ```usbipd attach --wsl --busid <id>``` to attach the USB port to WSL.
6. In WSL, you can use lsusb to see the device.

### Expose WSL port to Docker image
1. Run ```ls /dev``` or ```lsusb``` to view ports accessible by WSL. For me, the Teensy appears as ```/dev/ttyACM0```.
2. Run the following command in WSL to expose the shared port with the docker image:
```
docker run -it --rm --name ros2_container \
-v $(pwd)/src:/home/ros2_user/ros2_ws/src \
--device=<port>  ros2_humble_custom 
```





