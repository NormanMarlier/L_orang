# Lorang  [![Build Status](https://travis-ci.com/NormanMarlier/Lorang.svg?branch=master)](https://travis-ci.com/NormanMarlier/Lorang)
Lorang is a 3D-printing robotic arm made of PLA which integrates a low-level controller in Arduino board and a high-level controller in a Raspberry Pi.  



# Hardware
The mechanical structure is made by following these instructions [EEzyBotARM](https://www.thingiverse.com/thing:1454048). You can find the motors used for moving the arm.

I add hardware in order to control the robotic arm and interact with the environment :
* Arduino UNO board. It controls the motors and the emergency stop
* Raspberry Pi 3 which interacts with the world. It provides services such as inverse kinematic solver, joint trajectory, object recognition, ...
* A raspberry pi camera v2  

- [ ] Power supply for the servomotors   

# Software
Communication between the Raspberry Pi board and the Arduino board is done by using the middleware [ROS Kinetic](http://www.ros.org/).

# Result
Some images of Lorang

![Lorang_1](../master/Lorang_1.jpg)

![Lorang_2](../master/Lorang_2.jpg)



