
## Copter simulation in Linux
1. Go into the ArduCopter directory.

`cd && cd ardupilot/ArduCopter/`

2. Start the simulation with wiping the virtual EEPROM option to load correct parameters.

`sim_vehicle.py -w`

3. After simulation has started, press Ctrl+C and start the simulation normally.

`sim_vehicle.py --console --map`

[Source](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
