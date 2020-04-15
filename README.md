# ROSplane

This the fork of ROSplane used by the BYU AUVSI SUAS competition team, with various changes and added features to fulfill the requirements of that competition.

ROSplane is limited-feature fixed-wing autopilot built around ROS. It is intended to be flown with [ROSflight](https://rosflight.org) as the hardware I/O or in the Gazebo simulation environment.  It is built according to the method published in [Small Unmanned Aircraft: Theory and Practice](http://uavbook.byu.edu/doku.php) by Beard and McLain, so as to allow anyone to easily understand, modify and use the code.  This framework is inherently modular and extensively documented so as to aid the user in understanding and extending for personal use.

This repository features three ROS packages: rosplane, rosplane\_msgs, and rosplane\_sim. The contents of each of these three packages are described below.

To fly in hardware, ROSplane is intended to be used with [ROSflight](https://github.com/rosflight/rosflight) and a flight controller (F1 or F4) running the ROSflight [firmware](https://github.com/rosflight/firmware).

To fly in simulation, simply build these packages in a catkin workspace and launch fixedwing.launch:

`$ roslaunch rosplane_sim fixedwing.launch`

Note: To successfully build, it may be needed to clone [rosflight_plugins](https://github.com/byu-magicc/rosflight_plugins.git) and [ROSflight](https://github.com/rosflight/rosflight.git) into your catkin workspace. Additionally, retrieve the necessary ROSflight submodules with:

`cd rosflight/`

`git submodule update --init --recursive`

#Changes from upstream ROSplane
* rosplane\_msgs
	* Added `Extended_Path` and `Full_Path` messages. These are used with the changes to the path manager; see below.
* Estimator
    * Added the `ins_estimator` node, which uses an inertial sense, along with a barometer and airspeed sensor to provide state estimates.
* Controller
    * Added pitch feedforward: In a turn, the plane will pitch up to compensate for loss of lift from banking.
* Path Manager
    * Rewritten to separate generating paths from keeping track of the current path.
    * Added support for extended paths, which have additional information, such as which part of an arc will be flown, and how far a line will be flown. This additional information is only for user convenience, and the path manager does not use it. These are published to the `extended_path` topic.
    * Added support for full paths. Each time waypoints are added or removed, the path manager creates the whole path that will be followed. Again, this is not used by the path follower, but makes it much easier to see what the plane is doing. These are published to the `full_path` topic. Unlike the current path and extended path, these are only published when waypoints change.
    * Added the `do_fillets` parameter, which defaults to true. If it is true, it will add fillets to paths between waypoints that don't have heading information. If false, it will use straight lines, which will always be overshot.
    * In default ROSplane behavior, if there are no waypoints, it will orbit the origin. When a new waypoint it added, the current position of the plane is added as a waypoint, which allows the plane to fly to the next waypoint better, but is annoying in certain situations. We removed that behavior. With no waypoint, it still orbits the origin. With one waypoint, it will orbit that waypoint. At no point is the current position of the plane added as a waypoint, and so you will have to do that manually if you want that behavior.
    * 


# rosplane

rosplane contains the principal nodes behind the ROSplane autopilot. Each node is separated into a \_base.h/.cpp and a \_example.h/.cpp in an attempt to abstract the ROS code from the guidance and control code covered in [Small Unmanned Aircraft: Theory and Practice](http://uavbook.byu.edu/doku.php). Each node is described below.

## - Estimator 

The estimator is a standard extended Kalman Filter (EKF), implemented as defined in the above reference. It has an attitude filter and a position filter for gps smoothing. We are estimating position, velocity, and attitude. The state is then published in the rosplane_msgs/msg/State.msg.

## - Controller

Implements a nested PID controller according to the reference above.  Requires State and Controller_Commands messages to be published.  Altitude is controlled in a longitudinal state machine including take-off, climb, desend, and hold zones. Controller can be tuned using rqt_reconfigure.

## - Path Follower

Gets the aircraft onto a Current_Path using vector fields. Chi_inf along with K_path and K_orbit gains can also be tuned using rqt_reconfigure.

## - Path Manager

Receives Waypoint messages and creates a path (straight line, fillet, or Dubins) to acheive all the waypoints.

# rosplane_msgs

rosplane_msgs is a ROS package containing the custom message types for ROSplane. These message types include Controller_Commands, Controller_internals, Current_Path, State, and Waypoint.


# rosplane_sim

rosplane_sim contains all the necessary plugins to fly the ROSplane autopilot in the Gazebo simulation. Fixedwing.launch launches a basic simulation with the ROSplane autopilot. rosflight_sil.launch launches a software-in-the-loop simulation for a fixedwing running the ROSflight firmware. An example .yaml file is also included for the simulated airframe.

If you use this work in your research, please cite:
```
@INPROCEEDINGS{ellingson2017rosplane,
  author = {Ellingson, Gary and McLain, Tim},
  title = {ROSplane: Fixed-wing Autopilot for Education and Research},
  booktitle = {Unmanned Aircraft Systems (ICUAS), 2017 International Conference on},
  year = {2017}
  organization={IEEE}
}
```
