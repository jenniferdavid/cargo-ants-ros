//about sockets (inspiration): 
// http://www.bogotobogo.com/cplusplus/sockets_server_client.php
// http://www.binarytides.com/programming-udp-sockets-c-linux/

#ifndef udp_receiver_H
#define udp_receiver_H

//std C/C++
#include <iostream>
#include <iomanip>
#include <cstring> //memcpy()
#include<string.h> //memset()
#include <unistd.h> //close()
#include<arpa/inet.h> //socket constants
#include<sys/socket.h> //sockets

//at_udp2ros
#include "constants.h"

// //general purpose constants
// #define SDF_MODE 1
// #define LIDAR_MODE 2
// #define BUFFER_LENGTH 2048 //in Bytes
// #define STACKING_PACKETS 0
// #define MESSAGE_COMPLETE 1
// #define FIRST_PACKET 2
// 
// //SDF constants
// #define SDF_THIS_PORT 13000
// //#define SDF_SENDER_PORT 65533 //port from where sdf (vehicle state + radar) data is being sent
// #define SDF_SYNC_WORD 0xF0E1D200
// #define SDF_COUNT_INIT 0xC3 //starting value of sdf datagram counter
// #define SDF_DATAGRAM_PER_MSG 4
// #define SDF_DATAGRAM_LENGTH 1472 //367*4Bytes + 4Bytes (with sync word)
// #define SDF_MESSAGE_LENGTH 5872 //1468*4Bytes
// 
// //LIDAR constants
// #define LIDAR_THIS_PORT 2001
// //#define LIDAR_PORT 65530 //port from where lidar data is being sent
// #define LIDAR_SYNC_WORD 0xFEDCBA00
// #define LIDAR_COUNT_INIT 0x98 //starting value of lidar datagram counter
// #define LIDAR_DATAGRAM_PER_MSG 12
// #define LIDAR_DATAGRAM_LENGTH 1472 //367*4Bytes + 4Bytes (with sync word)
// #define LIDAR_MESSAGE_LENGTH 17616 //367*4*12Bytes

class UDPReceiver
{
	protected: 
		//sets SDF or LIDAR mode
		unsigned int mode_; 
		
		//port used by the socket to receive data
		int port_this_;
		
		//port from which udp packets are sent
		//unsigned short port_sender_;		

		//socket address associated to socket managed by this object 
		struct sockaddr_in si_this_; 
		
		//socket address associated to socket sending udp's 
		struct sockaddr_in si_sender_;    
		
		//socket file descriptor
		int socket_fd_; 
				
		//Sync word at the start of each udp packet
		unsigned int sync_word_; 

		//num of datagrams to build an entire message
		unsigned int datagram_per_msg_;
		
		//offset of the above counter
		unsigned int datagram_count_offset_; 
		
		//datagram_counter to build an entire message
		unsigned char datagram_count_;
		
		//datagram length
		unsigned int datagram_length_;
		
		//counter of total messages packed
		unsigned long total_messages_;
		
		//counter of total messages discarded due to packet loss
		unsigned long discarded_packets_;
		
	public:	
		//raw buffer. Bytes directly from udp socket
		//unsigned char raw_buffer_[BUFFER_LENGTH];
		unsigned char *raw_buffer_;

		//raw message. Stacked packets forming an entire message. Need to be allocated with proper length at the constructor
		unsigned char *raw_message_;
				
	public: 
		//constructor
		UDPReceiver(unsigned int _mode); 

		//destructor 
		~UDPReceiver();
		
		//returns packet counter
		unsigned char getPacketCounter();
		
		//init socket. Returns 1 if success, or -1 if error
		int initSocket(); 
		
		//Blocking reception. Returns number of bytes read if success, or -1 if error
		int blockingReception(); 
		
		//stack udp packets up to form a message. 
		//	Returns 1 if all packets are stacked and the message is ready to be decoded and published 
		//	Returns 0 if stack is not finished
		//	Returns -1 if error
		int packet2message(); 
};
#endif
