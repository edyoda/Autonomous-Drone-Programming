# Connecting Multiple UAV

This tutorial shows you how to model and control a multiple UAV using ardupilot

## Connecting Multiple Vehicles SITL 

```
-I0 for drone1
-I1 for drone2
-I2 for drone3

```
## Connecting Multiple Drones to a Ground Station

Each Drone in your swarm will be producing mavlink messages. In order to discern what message is from which drone, we will need to assign each drone a unique system ID. This is controlled by the parameter `SYSID_THISMAV`. 

## Launching Ardupilot SITL Instances with Unique Parameters

Usually when we launch the ardupilot sitl simulation we launch the instance using the below command
 
`-f` is used to specify the frame which Ardupilot will launch the simulation for. The frame type also specifies the location of the parameter files associated with the frame type. In order to simulate drones with different parameters we will need to create our own custom frames and parameter files.

First, we will want to edit the file `ardupilot/Tools/autotest/pysim/vehicleinfo.py` add the following lines in the SIM section.
```
            "copter-1": {
                "model": "+",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter-1.parm",
            },
            "copter-2": {
                "model": "+",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter-2.parm",
            },
            "copter-3": {
                "model": "+",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter-3.parm",
            },
```
We will then need to create the following files

- `default_params/copter-1.parm`
- `default_params/copter-2.parm`
- `default_params/copter-3.parm`

each with their corresponding `SYSID_THISMAV` parameter value ie
- `default_params/copter-1.parm` should contain `SYSID_THISMAV 1`
- `default_params/copter-2.parm` should contain `SYSID_THISMAV 2`
- `default_params/copter-3.parm` should contain `SYSID_THISMAV 3`

### Example copter-1.parm File
```
FRAME_CLASS 1
FRAME_TYPE  1
# IRLOCK FEATURE
RC8_OPTION 39
PLND_ENABLED    1
PLND_TYPE       3
# SONAR FOR IRLOCK
SIM_SONAR_SCALE 10
RNGFND1_TYPE 1
RNGFND1_SCALING 10
RNGFND1_PIN 0
RNGFND1_MAX_CM 5000
SYSID_THISMAV 1
```


## Connecting Multiple Drones to qgroundcontrol

In order to connect multiple unique vehicles to a ground station, you will need to make sure that the TCP or UDP connection ports do not conflict. For this example we will be using a TCP connection. The first thing we need to do is relaunch our SITL ardupilot instances with a unique in/out TCP port for our GCS. 

```
sim_vehicle.py -v ArduCopter -f copter-1 --console -I0 --out=udp:0.0.0.0:14550 -L UAV1 
```
```
sim_vehicle.py -v ArduCopter -f copter-2 --console -I1 --out=udp:0.0.0.0:14551 -L UAV2 
```

- note 0.0.0.0 allows any device on out local network to connect to the ardupilot instance 

### setup qgroundcontrol to accept multiple vehicles

navigate to the settings tab and click on `Comm Links`. then find the 

fill in each as bellow for the vehicle's unique UDP ports and connect

- note you can connect from a different device on the same network, by entering the ip address of the host computer in the host address box

### GPS Location coordinates check

- ardupilot/Tools/locations.txt
- add the Lat Lon values to the bottom of the page

```
UAV1=28.312562,77.352827,576.8,0
UAV2=28.312672,77.354393,576.8,0
```

