# Dockerized ROS2 Sailbot Setup

This tutorial will guide you through the steps to clone a ROS2 project repository via SSH, build a Docker image, and run it in a Docker container. By the end, you'll have a fully functional Docker environment set up for your ROS2 project.

---

## Prerequisites

Before we start, ensure the following prerequisites are met:
1. **Docker** is installed on your machine. You can download Docker Desktop [here](https://www.docker.com/products/docker-desktop).
2. You have **SSH keys** set up and added to your GitHub account. If not, follow [GitHub's guide on SSH key setup](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh).

---

## Step 1: Clone the Repository via SSH

1. Open your terminal and navigate to the directory where you want to clone the repository.
   
   ```bash
   cd ~/path/to/your/workspace
   ```

2. Clone the repository using SSH. Replace the SSH URL with the correct URL for your repository:

   ```bash
   git clone git@github.com:CUSail-Navigation/sailbot.git
   ```

   This will create a directory with the project files. Navigate to this directory:

   ```bash
   cd sailbot
   ```

Before building the Docker image, let's quickly go over the project structure. Your project has two main directories: `setup/` and `src/`.

- The `setup/` folder contains the `Dockerfile`, `requirements.txt`, and `setup_ros.sh` script used to configure your ROS2 environment in Docker.
- The `src/` folder holds all your ROS2 packages and code.

Here’s the file structure of the project:

```
/project-root
│
├── /setup
│   ├── Dockerfile
│   ├── requirements.txt
│   └── setup_ros.sh
│
└── /src
    └── (ROS2 packages)
```

### **Explanation of Files:**

- **`/setup/Dockerfile`**: Defines the Docker image used to create a containerized ROS2 environment. This includes setting up the base image, installing dependencies, and configuring the non-root user.
  
- **`/setup/requirements.txt`**: Contains a list of Python dependencies (if any) that will be installed inside the Docker container using `pip`. This file ensures that all necessary Python libraries are installed before running the ROS2 packages.
  
- **`/setup/setup_ros.sh`**: A shell script that automates the setup of your ROS2 workspace. It builds the ROS2 packages, sources the workspace, and keeps the container running interactively to test your code quicker.
  
- **`/src/`**: This directory holds your ROS2 packages, which contain the nodes, launch files, and any other necessary code for your ROS2 application. This is where the core functionality of our project resides.

---

With this structure, the `setup` folder contains all the configuration needed to build the Docker image and prepare the environment, while the `src` folder holds the ROS2 codebase that will be mounted into the Docker container for development and testing.

---

## Step 2: Build the Docker Image

1. Now, build the Docker image. You will need to run this command from the `setup` folder:

   ```bash
   cd setup
   docker build -t ros2_humble_custom .
   ```

   - `-t ros2_humble_custom`: Tags the image as `ros2_humble_custom`.
   - `.`: Tells Docker to use the current directory as the build context.

2. Docker will use the `Dockerfile` to create the image. This step may take a few minutes as Docker installs dependencies and sets up the environment.
3. Note that if you get an error like `ERROR: Cannot connect to the Docker daemon`, launch the Docker GUI application you installed and run this command again.

---

## Step 3: Run the Docker Container

Once the image is built, you can run the Docker container. Here's how to do it:

### **Run the Container with Volume Mounting**

You will need to run the following command from the project root to mount your `src/` directory to the container and make it accessible from within the container, use the following command:

```bash
cd ..
docker run -it --rm --name ros2_container \
  -v $(pwd)/src:/home/ros2_user/ros2_ws/src \
  ros2_humble_custom
```

**Explanation**:
- `-it`: Runs the container interactively, allowing you to enter commands.
- `--rm`: Automatically removes the container when it exits.
- `--name ros2_container`: Names the container `ros2_container`.
- `-v $(pwd)/src:/home/ros2_user/ros2_ws/src`: Mounts the `src/` directory from your host machine into the container at `/home/ros2_user/ros2_ws/src`.
- `ros2_humble_custom`: The name of the Docker image you built in Step 3.

### **Run a ROS Node**

To test that everything is working, you can run a ROS node from your workspace: 

```bash
ros2 launch sailboat_launch sailboat.launch.py
```

Note that this should fail trying to open the serial ports with a verbose error output, one line of which should look something like ```[airmar-1] serial.serialutil.SerialException: [Errno 2] could not open port /dev/ttyACM0: [Errno 2] No such file or directory: '/dev/ttyACM0'```

---

## Step 4: Stopping the Container

To exit the container, press `Ctrl+D` or type `exit`. If you used the `--rm` flag, the container will automatically be removed after you exit. If you want to keep the container running in the background, remove the `--rm` flag and instead use the `-d` flag for detached mode:

```bash
docker run -d --name ros2_container \
  -v $(pwd)/src:/home/ros2_user/ros2_ws/src \
  ros2_humble_custom
```

---

## Step 5: Understanding our Software Development Lifecycle (SDLC) Process

In this setup, the development process follows a typical Software Development Lifecycle (SDLC) workflow where you write and test your code iteratively. Here’s how it works with your ROS2 project:

1. **Developing Code in the `sailbot/src` Directory:**
   - You will write and edit your code directly in the `sailbot/src` directory, which contains your ROS2 packages. This is where all of your nodes, launch files, and configurations reside.
   - You are free to use any IDE or text editor you prefer (e.g., VS Code, PyCharm, or Sublime Text) to edit and manage your code. Since this directory is part of the local file system, your changes will be saved locally.

2. **Testing Your Code in the Docker Container:**
   - Once you have made changes and want to test them, you don’t need to worry about copying files into the container manually. The Docker setup automatically mounts your local `sailbot/src` directory into the Docker container. 
   - To start testing, simply run the Docker image with the command we defined earlier:

     ```bash
     docker run -it --rm --name ros2_container \
     -v $(pwd)/src:/home/ros2_user/ros2_ws/src \
     ros2_humble_custom
     ```

   - This will start the Docker container, mounting the `src` directory from your local machine, and build the workspace inside the container. From here, you can run any ROS2 nodes you wish to test.

3. **Live Changes and ROS Node Rebuild:**
   - **Live Changes:** Any modifications you make in the local `sailbot/src` directory will automatically be reflected inside the Docker container because of the mounted directory. This allows you to work and test your code seamlessly.
   - **Rebuilding ROS Nodes:** Although live changes are reflected in the container, changes to the code often require you to rebuild your ROS2 workspace inside the container. This can be done by running:
     
     ```bash
     colcon build
     ```

     After the build, you should source the setup file again:

     ```bash
     source install/setup.bash
     ```

     This ensures that the changes to your nodes or packages are properly built and ready for testing.
---

## Summary

By following this tutorial, you:
- Cloned a ROS2 project via SSH.
- Built a Docker image from the repository.
- Mounted the `src` directory into the Docker container.
- Built and sourced the ROS2 workspace within the container.

This setup allows you to work in a consistent environment without worrying about your host machine's setup. Docker ensures that all dependencies and tools are available and isolated in the container.

Happy ROS2 development!
