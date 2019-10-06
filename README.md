# Nao-Field-Limits-Recognition

A ROS based package implementation for Nao v5 robot to recognize and avoid goind out of bounds in C++.
The robot will be able to move freely inside a space created by lines.

### System Overview
![alt text](https://github.com/NekSfiris/Nao-Field-Limits-Recognition/blob/master/Images-Result/Diagrams-System%20Modelling/Untitled%20Diagram%20(2).jpg)

### Prerequisites

You need to have installed :
* OpenCV
* ROS ( with a catkin environment )
* naoqi_driver package

Has been tested for ros-kinetic and opencv 3.3.0 .

### Build

After you downloaded the package, move it in your catkin workspace and :
```
$ cd ~/catkin_ws
$ catkin_make
```

### Run

To run the package first run naoqi_driver in one terminal.
Then in another terminal : 
```
$ roslaunch localization_nao_pkg map_limits_recognition.launch
```
### System Outcome
![alt text](https://github.com/NekSfiris/Nao-Field-Limits-Recognition/blob/master/Images-Result/Nao-Movement/1.jpg)
