This package subscribes to an existing trajectory in the format ReferenceTrajectory.msg and encodes it into udp packets with doubles and sends it to the Autobox on the AT side with a specific IP and port address. As per AT requirements, each UDP packet is now = 20 points * 8 (doubles) * 9 parameters (x,y,th,xd,yd,thd,xxd,yyd,thdd) = 1440 bytes each. This may vary according to the trajectory points required by the AT.

    roscore                                               //on a separate terminal
    rosrun cargo_ants_path_adaptor path_adaptor           //this initiates path adaptor
    rostopic echo /trajectory                             //it echoes the trajectory developed by chomp
    rostopic pub -1 /path_planner cargo_ants_msgs/Path '{mode: 0, container: name , goals: [{gx: x0, gy: y0, gth: 0, dr: 0, dth: 0},{gx: x1, gy: g1, gth: 0, dr: 0, dth: 0}]}'                //Replace x0,y0 and x1,y1 (below) by coordinates of start and end points respectively.

    ./devel/lib/at_ros2udp/at_ros2udp                    //This package subscribes the trajectory and sends it to the UDP port
