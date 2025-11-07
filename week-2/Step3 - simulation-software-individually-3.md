# Starting the simulation software individually
1. Create a simulation directory in home.

`cd && mkdir -p ardu-sim/parameters`

2. Copy simulation software binaries and default parameters to the directory.

`cp -a $HOME/ardupilot/build/sitl/bin/. $HOME/ardu-sim/`

`cp $HOME/ardupilot/Tools/autotest/default_params/copter.parm $HOME/ardu-sim/parameters/copter.parm`

3. Get into the simulation directory.

`cd && cd ardu-sim/`

## Starting a copter simulation individually
`./arducopter -S --model + --speedup 1 --defaults parameters/copter.parm -I0`

[Source](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
