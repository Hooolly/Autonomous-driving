Technical Autonomous Systems WS2017/18 G3
=================================
This git presents the work developed in the frame of the TAS Lecture for the WS2017/18. 
To install this code make sure you have a functional catkin workspace. 
Then clone this git in your *catkin/src*.

## Contributors
+ Hao Xu
+ Fernando Espindola

## Contributions
### Task 1:
+ Autonomous driving on a pre-defined path. *Fernando Espindola & Hao Xu*

### Task 2:
+ Build simulation environment in Stage. *Fernando Espindola*
+ Implement slalom (with trailer) using method 1. *Fernando Espindola*
+ Implement slalom (with trailer) using method 2. *Hao Xu*
+ Trailer system analysis. *Hao Xu*

## Files contributed
### Fernando Espindola
Packages and all files within the following folders:
+ my_turtle/
+ slalom/
+ tf2odom/

### Hao Xu
Packages and all files within the following folder:
+ slalom_2/

## Dependencies and how to install
For installing our packages some libraries are necessary:
```sh
$ cd ~
~$ git clone --recursive https://github.com/libigl/libigl.git
~$ sudo apt install libeigen3-dev
```
The Eigen library is used for operations with vectors and matrices. We used them in the implementation of our algorithms for the slalom task.

For installing download this repository to your *~/catkin_ws/src* and proceed as follows:

```sh
~$ cd catkin_ws/src
~/catkin_ws/src$ git clone http://tas.lsr.ei.tum.de/TAS201718/TAS201718_G3.git
~/catkin_ws/src$ cd ..
~/catkin_ws$ catkin_make
```

## Autonomous driving
This task was solved by modifing some parameters mostly asociated to the navigation stack nodes.
Some of them were: `obstacle_range`, `inflation_radius` and `heading_lookahead`.

We encountered massive inconvenients due to changes in the hardware for Vettel. 
Apparently there was a change in the motor direction for the steering. After struggling we 
came out a solution in software, namely a sign inversion. 

```c++
cmd_steeringAngle = 1500 - 500/30*cmd_steeringAngle;
```

This issue should be known to avoid wasting working time. Tutors should know this unwanted effect.

## Simulation with Stage
The initial robot pose and the map can be changed in *my_turtle/world/maze.world*
After having launch the node called *myturtle_tas.launch* you can call gmapping or SLAM approaches as if you had the real car.
Move_base and the setting of simple navigation goals should also work. 
To change features of the map, just modify the available worlds with a program like gimp or even paint.
Black objects become obstacles in Stage. White pixels are taken as free areas.
Do not forget to change the following parameter in your launch files:
```html
<param name="/use_sim_time" value="true"/>
```
## Slalom driving method 1
+ Look for 1st pylon.
+ Transform point to frame /odom
+ Project point 4 times in x-direction
+ Broadcast tfs for every pylon
+ Define relative goals

## Slalom driving method 2
+ Detect all cones according to distance range and view angle
+ Calculate and publish waypoints 
+ Using trailer system to accomplish slalom with trailer

## Practical alias commands:
These commands can be appended to your *.bashrc* in order to save time and run nodes faster.

```sh
alias myturtle='roslaunch my_turtle myturtle_tas.launch'
alias odom='roslaunch tas odom_gmap.launch'
alias move_base='roslaunch tas move_base_gmap.launch'
```
