### Installation (PX4 + ROS2 + MicroXRCEDDS)

1.Download the PX4 Source Code 

The PX4 source code is stored on Github in the PX4/PX4-Autopilot repository.

To get the very latest (main branch) version onto your computer, enter the following command into a terminal:

```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

```

2. Run the ubuntu.sh with no arguments (in a bash shell) to install everything:

Now update all dependencies:

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

3. First Build (Using a Simulator) 

First we'll build a simulated target using a console environment. This allows us to validate the system setup before moving on to real hardware and an IDE.

Navigate into the PX4-Autopilot directory and start Gazebo SITL using the following command:

```
make px4_sitl gz_x500
```


4. Install from Snap Package:

Install from a snap package on Ubuntu using the following command:

```
sudo snap install micro-xrce-dds-agent --edge

```

To start the agent with settings for connecting to the uXRCE-DDS client running on the simulator (note that the command name is different than if you build the agent locally):

```
micro-xrce-dds-agent udp4 -p 8888
```


5. To build and run the example:

Open a new terminal.

Create and navigate into a new colcon workspace directory using:

```
mkdir -p ~/ws_offboard_control/src/
cd ~/ws_offboard_control/src/
```

Clone the px4_msgs repo to the /src directory (this repo is needed in every ROS 2 PX4 workspace!):

```
git clone https://github.com/PX4/px4_msgs.git
# checkout the matching release branch if not using PX4 main.
```

Clone the example repository px4_ros_com to the /src directory:

```
git clone https://github.com/PX4/px4_ros_com.git

```

Source the ROS 2 development environment into the current terminal and compile the workspace using colcon:

```
cd ..
source /opt/ros/humble/setup.bash
colcon build
```
Source the local_setup.bash:

```
source install/local_setup.bash
```

Launch the example.

```
ros2 run px4_ros_com offboard_control

```
