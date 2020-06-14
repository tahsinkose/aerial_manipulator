# Aerial manipulator

* Authors: zhonghang <zhonghang@hnu.edu.cn>, tahsinkose <tahsincankose@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)

Gazebo Simulation for quad manipulator using PX4

## Setup
Create a catkin workspace and clone the repo:

```
mkdir -p catkin_ws/src
cd src
git clone git@github.com:tahsinkose/aerial_manipulator.git
cd aerial_manipulator
```
Since this project employs PX4 Firmware, we have to first get the submodules.
```
git submodule update --init --recursive
```
After this, do the following to compile the firmware:

```
cd Firmware
source /opt/ros/*/setup.bash
DONT_RUN=1 make px4_sitl_default gazebo
```

`DONT_RUN=1` is prepended to skip launching empty Gazebo session, since we don't need it at this point. Followingly we need to
create a model specific file under `ROMFS/px4fmu_common/init.d-posix`. This is a must-do step for any new robot not officially supported by PX4 codebase. Create `Firmware/ROMFS/px4fmu_common/init.d-posix/10090_quad_manipulator` file with the following content:

```
#!/bin/sh

sh /etc/init.d/rc.mc_defaults

set MIXER quad_w
```

Next, we need to blacklist `px4` package, since it uses plain CMake for compilation and we don't want to mess with it via `catkin`. Also we need to extend the installation path of `melodic`:

```
catkin config --blacklist px4
catkin config --extend /opt/ros/melodic
```

and now compile with:

```
catkin build
```

## Quick Start

This phase is not so quick due to the convolved nature of PX4. Paste the following snippet to the terminal:

```
cd catkin_ws
source devel/setup.bash
cd src/quad_manipulator/Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
```

Finally run:

```
roslaunch quad_manipulator_gazebo simulate.launch
```


## Develop and Contribute

We welcome any contributions to this repo and encourage you to fork the project then send pull requests back to this parent repo. Thanks for your help!
