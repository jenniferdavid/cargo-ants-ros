
//rapid test with netcast
// $ nc -u 127.0.0.1 20002
// $ echo 'C3D2E1F0' | xxd -r -p | nc -p 13000 -u 127.0.0.1 20001

//std C/C++
#include <iostream> //cout
#include <cstdlib> //argc,argv

// ros
#include <ros/ros.h>

//local dependencies
#include "ros_publisher.h"
#include "udp_receiver.h"

//node main
int main(int argc, char **argv)
{
    //local vars
    ros::Time ts_prev_, stamp_;
    double delay_= 0;
    
    //init ros
    ros::init(argc, argv, "udp2ros_node");
    
    //ros node handle
    ros::NodeHandle nh_(ros::this_node::getName());
	
	//get operation mode from the user entry (TODO: check input correctness)
	int op_mode_ = atoi(argv[1]);
	
	//udp udp_receiver
	UDPReceiver udp_receiver_(op_mode_); 
    
    //ros publisher local class
    ROSPublisher ros_pub_(op_mode_);
    
    //main loop 
    while ( ros::ok() )
    {
        //do things
        //std::cout << std::endl << "Waiting reception ... PacketCounter: " << (unsigned int)udp_receiver_.getPacketCounter() <<  std::endl;
        
        //Receive some data, (blocking call)
		if ( udp_receiver_.blockingReception() != -1 )
		{
			switch(udp_receiver_.packet2message())
			{
				case FIRST_PACKET:
					stamp_ = ros::Time::now(); //keep the stamp from the first UDP packet
					break;

				case MESSAGE_COMPLETE:
					switch (op_mode_)
					{
						case SDF_MODE:
							//call to ros publisher
							ros_pub_.publishSdf(udp_receiver_.raw_message_, SDF_MESSAGE_LENGTH, stamp_);
							break;

						case LIDAR_MODE:
							//call to ros publisher
							if (ros_pub_.fillLidarData(udp_receiver_.raw_message_, LIDAR_MESSAGE_LENGTH))
							{
                                ros_pub_.publishScans(stamp_);
                                ros_pub_.publishPointCloud(stamp_);
							}
							break;

						default:
							break;
					}
					break;

				default:
					break;
			}
		}
	}
    
    //exit program
    return 0;
}
