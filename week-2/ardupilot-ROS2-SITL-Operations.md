### Installation (Ubuntu)

1. To make installation easy, we will clone the required repositories using vcs and a ros2.repos files:

```
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
```

2. This will take a few minutes to clone all the repositories your first time.

Now update all dependencies:

```
cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```


3. Installing the MicroXRCEDDSGen build dependency:

```
sudo apt install default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
```

4. Test microxrceddsgen installation:

```
source ~/.bashrc
microxrceddsgen -help
# microxrceddsgen usage:
#     microxrceddsgen [options] <file> [<file> ...]
#     where the options are:
#             -help: shows this help
#             -version: shows the current version of eProsima Micro XRCE-DDS Gen.
#             -example: Generates an example.
#             -replace: replaces existing generated files.
#             -ppDisable: disables the preprocessor.
#             -ppPath: specifies the preprocessor path.
#             -I <path>: add directory to preprocessor include paths.
#             -d <path>: sets an output directory for generated files.
#             -t <temp dir>: sets a specific directory as a temporary directory.
#             -cs: IDL grammar apply case sensitive matching.
#     and the supported input files are:
#     * IDL files.
```

5. If you have installed FastDDS or FastDDSGen globally on your system: eProsima’s libraries and the packaging system in Ardupilot are not deterministic in this scenario. You may experience the wrong version of a library brought in, or runtime segfaults. For now, avoid having simultaneous local and global installs. If you followed the global install section, you should remove it and switch to local install.
And finally, build your workspace:

```cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests ```

6. If the build fails, when you request help, please re-run the build in verbose mode like so:

```colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+ '''

7. If you’d like to test your ArduPilot ROS 2 installation, run:

```cd ~/ardu_ws
source ./install/setup.bash
colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
colcon test-result --all --verbose```

###ROS 2 with SITL


8. Once ROS2 is correctly installed, and SITL is also installed, source your workspace and launch ArduPilot SITL with ROS 2!

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:


```
source /opt/ros/humble/setup.bash
cd ~/ardu_ws/
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
transport:=udp4 \
synthetic_clock:=True \
wipe:=False \
model:=quad \
speedup:=1 \
slave:=0 \
instance:=0 \
defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm \
sim_address:=127.0.0.1 \
master:=tcp:127.0.0.1:5760 \
sitl:=127.0.0.1:5501
```


