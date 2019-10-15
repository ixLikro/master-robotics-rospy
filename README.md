# Master: Robotics
## Working with ROS (Robot Operating System)


### How to run this Code:
- [Install ROS Kinetic on Ubuntu 16.04.](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Create a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
- Build workspace:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
- Clone this Repo inside **~/catkin_ws/src**:
```
cd ~/catkin_ws/src
git clone https://github.com/ixLikro/master-robotics-rospy.git
```
- Do you want to work with gazebo and turtlebot3?  Then install ros dependencies and clone some turtlebot3 repositorie:
```
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
For more information see [here](https://github.com/saimouli/frontier_exploration_turtlebot#turtlebot3-packages-installation) or the [eManuel](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo)
- Build again:
```
cd ~/catkin_ws
catkin_make
```

After a successful build you can start nodes. For Example the hello world turtle Example:

Run in individual Terminals:
```
roscore
```
```
rosrun turtlesim turtlesim_node
```
```
rosrun hello_world Turtle_draw_house.py
```

Or a gazebo simulation (individual Terminals):
```
roscore
```
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
```
rosrun hello_world TurtleBot3_drive.py
```