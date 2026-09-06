# Overview

The `teensy` directory includes the code for our Teensy 4.0 microcontroller, which serves as the low-level controller 
of our sailboat. The code manages the sensors, actuators (such as the servos for the sail and rudder), and communicates 
with the Jetson via a serial connection.


## Code Overview
This code is structured based on [Lodestar](https://github.com/shihaocao/lodestar), a small scale electric demonstrator for the belly-flop and 
tail-sitting control algorithms necessary for SpaceX's Starship.

### main.cpp
This file is comparable to a .ino file you would see in the Arduino IDE (notice setup and loop are exactly the same as 
they would be in an Arduino file).

### MainControlLoop.cpp
The MainControlLoop initializes and executes all monitors and control tasks.

### SFR
SFR stands for State Field Registry. It contains values that should be available to the entire boat (sensor values, 
serial buffer data, etc).

### Monitors
Monitors read input from some source and update sensor values in the SFR.

### Control Tasks
Control tasks perform actions based on the current state of the boat or SFR values.

### constants.hpp
This file contains values that will never be changed. This prevents "magic numbers" in the codebase.


## Getting Started
Below are the steps to set up your development environment to upload code and observe serial outputs from the Teensy.

### Prerequisites: 
- [VSCode](https://code.visualstudio.com/download) or [CLion](https://www.jetbrains.com/clion/) is installed.
- The [sailbot](https://github.com/CUSail-Navigation/sailbot) repository is cloned.

### Steps:
1. In VSCode, click "Extensions" on the left-hand side toolbar and search for PlatformIO IDE.  
   In CLion, click "File → Plugins" and search for PlatformIO for CLion.
2. Open the `teensy/` folder within the sailbot repository. Make sure the `teensy/` folder is the project root.
3. (VSCode) At the bottom of your screen in the blue toolbar, you should see a check, arrow, and serial monitor icon.
   - If you would just like to compile code but not upload to the Teensy, press the check. 
   - If you would like to upload to the Teensy, press the arrow. 
   - To view the serial monitor, press the electrical cord icon.


## Developing with a Teensy with Docker & WSL on Windows:
The following steps expose a Windows COM port to WSL and then expose the WSL port to the Docker image running in WSL. 
This was necessary to set up a test environment with the ROS2 codebase on a Windows 11 computer.

### Changing Docker Desktop Backend to support WSL
Make sure you have:
- [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/) installed.
- [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) installed.

1. Open Docker Desktop.
2. Navigate to Settings → General.
3. Check the box "Use the WSL 2 based engine".

### Expose Windows COM port to WSL 
1. Download and install [USBIPD-WIN](https://github.com/dorssel/usbipd-win/) (follow their README.md instructions).
2. Open PowerShell as an administrator.
3. Obtain a list of USB devices using `usbipd list`.
4. Find the bus ID of the device (e.g. 4-4) and use `usbipd bind --busid <id>` to share it with WSL.
5. Use `usbipd attach --wsl --busid <id>` to attach the USB port to WSL.
6. In WSL, you can use the command `lsusb` to see the device.

### Expose WSL port to Docker image
1. Run `ls /dev` or `lsusb` to view ports accessible by WSL. The Teensy will most likely appear as `/dev/ttyACM0`.
2. Run the following command in WSL to expose the shared port with the docker image:
```
docker run -it --rm --name ros2_container -v $(pwd)/src:/home/ros2_user/ros2_ws/src --device=<port>  ros2_humble_custom 
```





