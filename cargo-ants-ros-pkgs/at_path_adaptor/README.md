# Cargo-ANTS AT Path Adaptor
CHOMP implementation is based on source code from [trychomp][] which is based on [CHOMP][].



## Compile:

catkin_make --only-pkg-with-deps cargo_ants_at_path_adaptor

 

## Usage:

```c++
void generatePath(VectorXd const &qs, VectorXd const &qe, VectorXd &xi, MatrixXd const &obs);
```

- qs : Start point (x,y) as a 2x1 vector.
- qe : End point (x,y) as a 2x1 vector.
- xi : Generated chomp trajectory points (x<sub>1</sub>,y<sub>1</sub>,x<sub>2</sub>,y<sub>2</sub>,...,x<sub>N</sub>,y<sub>N</sub>) as 2*Nx1 vector. N is currently fixed to 20.
- obs : A matrix of disk like obstacles of size Kx3. Each row is (x,y,R) of the obstacle.

Notes:
- If the function is called with a non empty trajectory xi, the function will use the provided trajectory as an initial guess for the chomp algorithm.
- demo/minimal_program.cpp shows a basic example of using chomp

## ROS Support:
You need [Cargo-ANTS][] for messages definitions. Place the project folder inside  (path_to_ros_workspace)/src and then build by executing catkin_make.<br>For testing:

    roscore
    //on a separate terminal
    rosrun at_path_adaptor at_path_adaptor
    //on a separate terminal
    rostopic echo /trajectory  
    //on a separate terminal. Replace x0,y0 and x1,y1 (below) by coordinates of start and end points respectively.
    rostopic pub -1 /path_planner cargo_ants_msgs/Path '{mode: 0, container: name , goals: [{gx: x0, gy: y0, gth: 0, dr: 0, dth: 0},{gx: x1, gy: g1, gth: 0, dr: 0, dth: 0}]}'
    //on a separate terminal. Replace o0,o0,R0 and o1,y1,R1 (below) by coordinates x,y,Radius of obstacles
    rostopic pub -1 /obstacles cargo_ants_msgs/ObstacleMap '{obstacles:[origin: {ox : x0, oy : y0 , oth : R0}, origin:{ox : x1, oy : y1, oth : R1}]}'



[cmake]: http://cmake.org/
[eigen]: http://eigen.tuxfamily.org/
[gtk+]: http://www.gtk.org/
[chomp]: http://www.nathanratliff.com/research/chomp
[trychomp]: https://github.com/poftwaresatent/trychomp
[cargo-ants]: https://github.com/jenniferdavid/cargo-ants-ros
