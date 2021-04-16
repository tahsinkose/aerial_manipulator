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
source /opt/ros/melodic/setup.bash
DONT_RUN=1 make px4_sitl_default gazebo
```

`DONT_RUN=1` is prepended to skip launching empty Gazebo session, since we don't need it at this point.

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

Go into your catkin workspace. Then do the following.

```
cd catkin_ws
source devel/setup.bash
```

Finally run:

```
roslaunch quad_manipulator_gazebo generic_test.launch
```

## Troubleshooting

1. "preflight check failed, missing sensor during GAZEBO simulation" -> solve by removing `~/.ros/eeprom/parameters` file.


## Develop and Contribute

We welcome any contributions to this repo and encourage you to fork the project then send pull requests back to this parent repo. Thanks for your help!
