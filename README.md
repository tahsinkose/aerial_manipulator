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
to build the container. Similarly, the name of container is hardcoded as `aerial-manipulation-melodic-container`. Next enter into the container:

```
./run_container.sh
```

Now compile PX4:
```
make px4_sitl
```

## Aerial Manipulation Stack Setup
Spawn a new terminal. This time run `./enter_container` bash script. Here, we need to blacklist `px4` package, since it uses plain CMake for compilation and we don't want to mess with it via `catkin`. Besides it has been already compiled in the **Docker Setup** section. Finally, we need to extend the installation path of `melodic`:

```
cd /quad_manip_ws
catkin config --blacklist px4
catkin config --extend /opt/ros/melodic
```

Next, install all dependencies in the workspace. Notice that we need to skip `mavros` and `mavros_msgs` packages as they will be custom built in the next stage:
```
rosdep update
rosdep install --from-paths src --ignore-src --skip-keys="mavros mavros_msgs libmavconn mavros_extras" -r -y
```

and now compile with:

```
catkin build
```

Then source the workspace layout for ROS:

```
source devel/setup.bash
```

In order to bringup an idle session of a single robot, run:

```
roslaunch quad_manipulator_gazebo sim_single_uav.launch
```

With this command you should have a running Gazebo simulator with aerial manipulator on the ground. The next step is to connect `QGroundControl` to the robot. This is unfortunately not straightforward at the moment and requires inspection of two parameters:
1. We need to figure out the UDP port of quadcopter. It can be found in `build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink` on the parameter `udp_gcs_port_local`.
2. The IP address of the container we just created -> `docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}'  aerial-manipulation-melodic-container`

Having both parameters, navigate to Application Settings (opens new window) and select Comm Links in QGroundControl. Create a new link that uses the UDP protocol. Specify the value you retrieved in Step-1 above to *Listening Port* and  the value from Step-2 to *Target Hosts*. After adding this link, click *Connect*. You can now use QGroundControl as if you would normally be doing in the host.

## Troubleshooting

1. "preflight check failed, missing sensor during GAZEBO simulation" -> solve by removing `~/.ros/eeprom/parameters` file.

