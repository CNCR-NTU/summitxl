# ros_simple_follower
See it in action: https://youtu.be/XpQpfcJzsFU

A very simple implementation for ROS to make a mobile robot follow a target. Either using a Laser Range Finder to follow the closest object or an RGB-D camera to follow a specific color. It was developed for a Robotnik 'Summit XL' robot but should work anywhere (You might have to change the topics in the python files though). 

The visual tracker might be of some interest as an example for a very basic and naive approach at tracking colored 3D objects. 

## Instalation
Download the simple_follower folder into the src folder in your catkin workspace. e.g.:
```
roscd ; 
cd ../src
git clone https://github.com/rauwuckl/ros_simple_follower.git
cd ..
catkin_make
```

## Usage
`roslaunch simple_follower laser_follower.launch` or `roslaunch simple_follower visual_follower.launch` respectively. 

