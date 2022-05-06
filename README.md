# Running ROS simulation in GitHub Actions

Source code for the blog post with the same name.

## Design

Simple hello world ROS robot that moves around randomly. Has a launch file for Gazebo simulation, and for testing that the robot is moving around.

The project structure is as follows:

- `catkin_ws/` — ROS workspace.
- `Makefile` — Convenience build/run tasks.

## Installation and pre-requisites

You'll need access to a ROS Melodic environment. You can do this two ways:

1. Using Docker with included Makefile
2. Installing ROS Melodic yourself on Ubuntu — http://wiki.ros.org/melodic/Installation/Ubuntu

After you have a ROS environment, install the turtlebot packages:

```sh
sudo apt-get update && sudo apt-get install -y ros-melodic-turtlebot3-simulations
```

## How to run ROS Docker container

Setup:

```sh
make setup
```

Attach shell:

```sh
make shell
```

To stop:

```
make stop
```

## How to run simulation

Enter the docker shell, or on your local Ubuntu system, then:

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_turtlebot_sim my_turtlebot_sim.launch
```

