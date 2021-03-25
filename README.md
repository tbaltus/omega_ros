# omega_ros
The following package offers a ROS interface with the haptic device omega 7 from force dimension. This allows easy integration of the device within a ROS ecosystem.

1st download the official library from force dimension 

```
https://www.forcedimension.com/software/sdk#:~:text=The%20Force%20Dimension%20SDK%20is,Haptic%20SDK%20and%20Robotic%20SDK. 
```

You will find documentation about available functions at 

```
file:///home/rehassist/sdk-3.11.1/doc/dhd/index.html
file:///home/rehassist/sdk-3.11.1/doc/drd/index.html
```

In the bin folder of the official library run

```
sudo ./autoinit
```

This will initialize your device and allow you to activate the force mode with dhdEnableForce(DHD_ON);

# Cloning the package into your catkin_workspace

Clone the following package in the src folder of your catkin workspace

```
git clone https://github.com/tbaltus/omega_ros
```

# Running/Launching

Run `catkin_make` first

You can either launch the node

```
roslaunch omega_ros omega_force_dimension.launch
```

or run it

```
roscore
rosrun omega_ros omega7
```

# Topics of interest

Your can apply cartesian forces to the end-effector through the topic

```
/EE_cartesian_force
```

and retrieve positions and velocities of the end-effector

```
/EE_cartesian_position
/EE_cartesian_velocity
```