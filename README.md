# ROS_Quadcopter_EKF

Using ROS **noetic**

## Quadcopter simulation

- PX4 (meh, doesn't work with ROS noetic)
- Webots
- Gazebo vs Webots
- DJI Mavic 2 PRO
- Crazyflie
- [Standard ROS controller](http://docs.ros.org/en/noetic/api/webots_ros/html/index-msg.html) for Webots
- [Examples](https://github.com/cyberbotics/webots_ros) ROS interactions with Webots
- [Samples](http://wiki.ros.org/webots_ros/Tutorials/Sample%20Simulations) ROS with Webots
- [Tutorial](https://cyberbotics.com/doc/guide/tutorial-9-using-ros) of using ROS with Webots

## DJI Mavic 2 PRO

- Quadcopter
- [Webots](https://cyberbotics.com/doc/guide/mavic-2-pro#movie-presentation)
- Controller can be a rosnode
- Obtain position bsofjnfmsdjnfksndfjksdkfnsdf

## Crazyflie

- Quadcopter
- [Webots](https://cyberbotics.com/doc/guide/crazyflie)

## RF Beacon

- Place [emitter](https://www.cyberbotics.com/doc/reference/emitter?version=master&tab-language=c++) in Webots world
- Place [receiver](https://www.cyberbotics.com/doc/reference/receiver?version=master) in quadcopter
- receiver channel: -1, get signal from every channel available
- beacon channel: x + 1, x >= 0, know which signal is from which beacon
- How to link ROS to this? Don't miss the next episode...

### Receiver

- use **signalStrengthNoise** parameter and **wb_receiver_get_signal_strength** function to determine distance to the beacon
- **directionNoise** and **wb_receiver_get_emitter_direction** to get direction --> **explore this**; The noise is not dependent on the distance between emitter-receiver

### Scenarios

- Beacon blocked by object
- Signal out of range
- Using a [Supervisor](https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor?tab-language=python) to dynamically change the simulation environment

## ROS Integration

- Revisit nodes, topics and services
- Launch Webots
- Connect to robot model
- rviz
- Terrible documentation, yey. [ROS2](https://docs.ros.org/en/rolling/Tutorials/Advanced/Simulators/Webots.html) tutorial, hopefully it helps

## Extended Kalman Filter

Using [FilterPy](https://filterpy.readthedocs.io/en/latest/) Python package
