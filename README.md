# Aerial manipulator

* Authors: tahsinkose <tahsincankose@gmail.com>
* License: Proprietary

Software stack for quad manipulator using PX4

## Setup
Create a catkin workspace and clone the repo:

```
mkdir -p catkin_ws/src
cd src
git clone git@github.com:tahsinkose/aerial_manipulator.git
cd aerial_manipulator
git submodule update --init --recursive
```


## Docker Setup
After cloning the repo into a catkin workspace, it is necessary to build the Docker container. The Docker image name is hardcoded as `aerial-manipulation-melodic`, but you can easily parametrize that. Make sure that you are inside `aerial_manipulator` folder. Then run:

```
./build_image.sh
```
to build the container. Similarly, the name of container is hardcoded as `aerial-manipulation-melodic-container`. Next, we will verify the sanity of the Docker container:

```
./run_container.sh
make px4_sitl_default gazebo
```

With this command you should have a running Gazebo simulator with Iris quadcopter on ground. The next step is to connect `QGroundControl` to the robot. This is unfortunately not straightforward at the moment and requires inspection of two parameters:
1. We need to figure out the UDP port of quadcopter. It can be found in `build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink` on the parameter `udp_gcs_port_local`.
2. The IP address of the container we just created -> `docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}'  aerial-manipulation-melodic-container`

Having both parameters, navigate to Application Settings (opens new window) and select Comm Links in QGroundControl. Create a new link that uses the UDP protocol. Specify the value you retrieved in Step-1 above to *Listening Port* and  the value from Step-2 to *Target Hosts*. After adding this link, click *Connect*. You can now use QGroundControl as if you would normally be doing in the host.

## Aerial Manipulation Stack Setup
Finish the previous simulation session from **Docker Setup** section if it is still running.

We need to blacklist `px4` package, since it uses plain CMake for compilation and we don't want to mess with it via `catkin`. Besides it has been already compiled in the **Docker Setup** section. Finally, we need to extend the installation path of `melodic`:

```
catkin config --blacklist px4
catkin config --extend /opt/ros/melodic
```

Next, install all dependencies in the workspace:
```
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
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

In order to bringup an idle session of a single robot, run:

```
roslaunch quad_manipulator_gazebo sim_bringup.launch
```

## Troubleshooting

1. "preflight check failed, missing sensor during GAZEBO simulation" -> solve by removing `~/.ros/eeprom/parameters` file.

