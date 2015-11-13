# Gazebo Scene Descriptor for CHOMP
This ROS packages reads published Models State in a Gazebo world and translate them to Obstacle Map (for obstacles) and path(for goal) as defined in Cargo-ANTS projects.

## Build and Run:
You need the following additional ROS packages.
- [Cargo-ANTS Messages][]

Once all packages are placed inside catkin workspace:
```
cd location\of\catkin_ws
catkin_make
```
Notes:
- Only objects with names prefixed "Obs" are reported.
- Goal is read from any object with the name "Goal_Cone".

[Cargo-ANTS Messages]: https://github.com/jenniferdavid/cargo-ants-ros
