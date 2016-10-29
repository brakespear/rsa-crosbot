### Copyright School of Computer Science and Engineering, UNSW

### Welcome to the crobot package
> * Warning this package requires that you also have installed **hector_slam** located at https://github.com/tu-darmstadt-ros-pkg/hector_slam
The **rsa-crosbot** package contains navigation functionalities intended to be used in conjunction with the **rsa-ass2** package located at https://github.com/brakespear/rsa-ass2 to achieve some basic tasks of the Robocup@Home competition.
This package is largely unchanged from the one located here http://robolab.cse.unsw.edu.au:4443/rescue/crosbot. The main additions are in crosbot_explore where code has been written to set the mode of the explorer, change wallfollowing side and use the hector mapping.

## Hardware
This package has been used on Turtlebots only. However it should work on any mobile platform equipped with a Laser Range Finder. The topics would likely be different on a different platform.

## Installation
This instalation process is for **catkin** 
Assuming that your catkin workspace is under **~/catkin_ws**, if not replace **~/catkin_ws** with appropriate location. It also assumes you're running Bash shell, if you're running Zsh, source appropriate **setup.zsh** file.
```
cd ~/catkin_ws/src
git clone https://github.com/brakespear/rsa-crosbot.git
mv rsa-crosbot crosbot
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Running the exploration code
First run the base explorer - this includes the SLAM
```
roslaunch crosbot_explore explore.launch
```
Use the python mode client to select an initial wallfollowing side and pause the robot.
For example to start the robot left wallfollowing
```
rosrun crosbot_explore set_mode_client.py 2
rosrun crosbot_explore set_mode_client.py 0
```
Now run the wallfollow side switcher and start the robot
``` 
rosrun crosbot_explore set_wallfollow_side
rosrun crosbot_explore set_mode_client.py 1
```


