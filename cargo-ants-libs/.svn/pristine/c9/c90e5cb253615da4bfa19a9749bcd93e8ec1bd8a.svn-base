# CHOMP Based Navigation Stack for Turtlebot
![alt tag](https://cloud.githubusercontent.com/assets/9220666/10293485/08b91ef4-6bb5-11e5-8c23-2681b399a191.png)
##  

ROS Navigation stack for Turtlebot utilizing [CHOMP][] for trajectory generation and [Pose Feedback Controller][] for robot control. This is built as a part of Cargo-ANTS projects and utilize several packages from the project.

## Build and Run:
You need the following additional ROS packages.
- [Cargo-ANTS Path Adaptor][]
- [Chomp Scene Descriptor][]
- [Cargo-ANTS Messages][]

Once all packages are placed inside catkin workspace:
```
cd location\of\catkin_ws
catkin_make
roslaunch chomp_nav chomp_nav_demo.launch
```
You should see Gazebo simulator running with Turtlebot and some obstacles (construction barrels). To change current goal move the traffic cone.
## Issues:
- Visualization of trajectory points is messy. The goal markers are updated via SetModelState service but Gazebo is dropping some calls.
- Trajectory is generated only when the goal point is changed (by moving the traffic cone). Moving obstacles will not update trajectory even if the obstacle collides with the current trajectory.
- Ground Truth data (from Gazebo published model_states message) instead of odometry is used  as no SLAM solution is implemented.
- Turtlebot can spin out of control while executing high curvature maneuvers. This can be addressed at the trajectory generation level (by setting constrains on curvature) or at Pose Feedback level (by linking Robot linear velocity with curvature).



[CHOMP]: http://www.nathanratliff.com/research/chomp
[Cargo-ANTS Path Adaptor]: https://github.com/j3sq/ROS-CHOMP
[Cargo-ANTS Messages]: https://github.com/jenniferdavid/cargo-ants-ros
[CHOMP Scene Descriptor]: http
[Pose Feedback Controller]: https://mitpress.mit.edu/books/introduction-autonomous-mobile-robots
