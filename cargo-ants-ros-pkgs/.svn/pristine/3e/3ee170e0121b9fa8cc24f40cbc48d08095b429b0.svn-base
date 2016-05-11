Package for running AT ros2udp...

To run this separately:

roscore
rosrun cargo_ants_at_path_adaptor at_path_adaptor   //run the adaptor
rostopic pub /path_planner cargo_ants_msgs/Path -r 1 -- '{mode: 0, container: name , goals: [{gx: 0, gy: 0, gth: 0, dr: 0, dth: 0},{gx: 5, gy: 11, gth: 0, dr: 0, dth: 0}]}'    // give a path the adpator
roslaunch at_tests_real at_local_mapping.launch    //starts the mapping and sensors  
rosrun cargo_ants_udp cargo_ants_udp <ipaddress> <portno>    //starts the udp





