# quad manipulator

* Authors: zhonghang <zhonghang@hnu.edu.cn>, tahsinkose <tahsincankose@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)

Gazebo simulation for quad manipulator using px4

## Setup
Since this project employs PX4 Firmware, we have to first get the submodules.
```
git submodule update --init --recursive
```
After this do the following to compile the firmware:

```
cd Firmware
source /opt/ros/*/setup.bash
DONT_RUN=1 make px4_sitl_default gazebo
```

`DONT_RUN=1` is prepended to skip launching empty Gazebo session, since we don't need it at this point.

## Quick Start



## Develop and Contribute

We welcome any contributions to this repo and encourage you to fork the project then send pull requests back to this parent repo. Thanks for your help!
