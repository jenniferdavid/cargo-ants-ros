The src folder has UDPClient.cpp, UDPMessage.cpp and UDPServer.cpp which are the basic UDP scokets.
The other 4 .cpp files are used for converting ROS messages to/from UDP packets.



test_node_udp2ros.cpp
---------------------
Usage: ./test_node_udp2ros $portno$

Creates a UDP Server on the $portno$

Extra feature on ROS side:
In this code, we also publish a ROS msg with trajectory points (x,y,th,xd,yd,thd,xdd,ydd,thdd,t) as a UDP msg
 



test_feed_udp2ros.cpp
---------------------
Usage: ./test_feed_udp2ros $hostname$ $portno$

Creates a UDP client to the $portno$ mentioned
This creates a UDP message with some integers and reals.



test_node_ros2udp.cpp
---------------------
Usage: ./test_node_ros2udp $hostname$ $portno$

Creates a UDP client to the $portno$ 
This creates a UDP message with some integers and reals.

Extra feature on ROS side:
In this code, we subscribe to the ROS msg with trajectory points (x,y,th,xd,yd,thd,xdd,ydd,thdd,t) as a UDP msg


test_udp
--------
This .cpp is for testing UDP messages
 

Instructions:

roscore   (on ROS side)

./test_node_udp2ros 6000   (on ROS side)

./test_feed_udp2ros cargoants 6000  (on UDP side)

./test_node_ros2udp cargoants 6000    (on ROS side)
