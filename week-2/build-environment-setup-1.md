# Setting up the build environment in Linux

1. Update and upgrade software packages.

`sudo apt-get update && sudo apt-get upgrade`

2. Install git to the computer.

`sudo apt-get install git gitk git-gui`

3. Clone the ardupilot repository from github.

`cd && git clone https://github.com/ArduPilot/ardupilot.git`

4. Perform submodule updates.

`cd ardupilot/`

`git submodule update --init --recursive`

5. Install required packages.

`Tools/environment_install/install-prereqs-ubuntu.sh -y`

6. Update profile.

`. ~/.profile`

7. start and run SITL

`cd ardupilot/Arducopter`

`sim_vehicle.py`

[Source](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
