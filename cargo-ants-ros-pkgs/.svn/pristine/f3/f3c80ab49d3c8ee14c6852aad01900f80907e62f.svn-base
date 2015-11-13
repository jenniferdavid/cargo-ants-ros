#include "udp_receiver.h"

UDPReceiver::UDPReceiver(unsigned int _mode) :
	mode_(_mode),
	datagram_count_(0),
	total_messages_(0),
	discarded_packets_(0)
{
	//allocate raw buffer
	raw_buffer_ = (unsigned char*) malloc (BUFFER_LENGTH);
	
	//set variables according mode, and allocates raw message vector
	switch(mode_)
	{
		case SDF_MODE:
			port_this_ = SDF_THIS_PORT; 
			//port_sender_ = SDF_SENDER_PORT;
			sync_word_ = SDF_SYNC_WORD;
			datagram_per_msg_ = SDF_DATAGRAM_PER_MSG;
			datagram_count_offset_ = SDF_COUNT_INIT; 			
			datagram_length_ = SDF_DATAGRAM_LENGTH;
			raw_message_ = (unsigned char*) malloc (SDF_MESSAGE_LENGTH);
			break; 
			
		case LIDAR_MODE: 
			port_this_ = LIDAR_THIS_PORT; 
			//port_sender_ = LIDAR_SENDER_PORT;
			sync_word_ = LIDAR_SYNC_WORD;
			datagram_per_msg_ = LIDAR_DATAGRAM_PER_MSG;
			datagram_count_offset_ = LIDAR_COUNT_INIT; 			
			datagram_length_ = LIDAR_DATAGRAM_LENGTH;
			raw_message_ = (unsigned char*) malloc (LIDAR_MESSAGE_LENGTH);
			break; 
			
		default: 
			break; 
	}
	
	//opens and init socket
	initSocket(); 
}

UDPReceiver::~UDPReceiver()
{
	//close socket resource
 	close(socket_fd_);
	
	//frees allocated memory TODO: Seems that we change the pointer provided by malloc, so
	free(raw_buffer_);
	free(raw_message_);	
}

unsigned char UDPReceiver::getPacketCounter()
{
	return datagram_count_;
}

int UDPReceiver::initSocket()
{
	socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if ( socket_fd_ == -1 )
    {
        std::cout << "Unable to create a valid socket." << std::endl;
        return -1; 
    }
	
	//init addr structs
    memset((char *) &si_this_, 0, sizeof(si_this_));
    si_this_.sin_family = AF_INET;
    si_this_.sin_port = htons(port_this_);
    si_this_.sin_addr.s_addr = htonl(INADDR_ANY);    
    
    //bind socket to the port
    int bind_rv = bind(socket_fd_ , (struct sockaddr*)&si_this_, sizeof(si_this_) );
    if ( bind_rv < 0 )
    {
        std::cout << "Unable to bind a socket to port." << port_this_ << std::endl;
        return -1; 
    }

    //Success message and return
    std::cout << "UDP socket successfully created and bound to port " << port_this_ << "." << std::endl;
	return 1; 
 
}

int UDPReceiver::blockingReception()
{
	//Receive some data, (blocking call)
	unsigned int slen= sizeof(si_sender_);
	int recv_len = recvfrom(socket_fd_, (char*)raw_buffer_, BUFFER_LENGTH, 0, (struct sockaddr *) &si_sender_, &slen);

	//check general comm errors
	if ( recv_len == -1 )
	{
		std::cout << "UDP reception error" << std::endl;

		//reset counter to resync again
		datagram_count_ = 0; 
		
		//count as discarded message
		discarded_packets_ ++;
		
		//return error
		return -1; 
	}

	//check total packet length
	if ( recv_len != datagram_length_ )
	{
		std::cout << "Unexpected UDP packet length" << std::endl;
		std::cout << "\tReceived UDP packet length: " << recv_len << std::endl;
		std::cout << "\tExpected UDP packet length: " << datagram_length_ << std::endl;

		//reset counter to resync again
		datagram_count_ = 0; 
		
		//count as discarded message
		discarded_packets_ ++;		
		
		//return error
		return -1; 
	}
	
	//on success, return number of read bytes
	return datagram_length_; 
}

int UDPReceiver::packet2message()
{
	//check port sender
// 	if ( port_sender_ != ntohs(si_sender_.sin_port) )
// 	{
// 		std::cout << "Unexpected port number" << std::endl; 
// 		std::cout << "\tReceived packet from " << inet_ntoa(si_sender_.sin_addr) << ":" << ntohs(si_sender_.sin_port) << std::endl;
// 		std::cout << "\tWas expected from port: " << port_sender_ << std::endl;   
// 		
// 		//reset counter to resync again
// 		datagram_count_ = 0; 
// 		
// 		//return error
// 		return -1; 
// 	}
		
	//check sync word
	unsigned int sync = raw_buffer_[3] << 24 | raw_buffer_[2] << 16 | raw_buffer_[1] << 8 | raw_buffer_[0];
	if ( (sync & 0xffffff00) != sync_word_ ) 
	{
		std::cout << "Unexpected sync word." << std::endl; 
		std::cout << "\t Received: " << std::hex << (sync & 0xffffff00) << std::dec << std::endl;
		std::cout << "\t Expected: " << std::hex << sync_word_ << std::dec << std::endl; 
	
		//reset counter to resync again
		datagram_count_ = 0; 
		
		//count as discarded message
		discarded_packets_ ++;
				
		//return error
		return -1; 
	}
	
	//check packet counter (last byte of sync word)
	if ( (sync & 0x000000ff) != (datagram_count_ + datagram_count_offset_) ) //check counter
	{
		std::cout << "Unexpected packet counter." << std::endl; 
		std::cout << "\t Received: 0x" << std::hex << (sync & 0x000000ff) << std::dec << std::endl;
		std::cout << "\t Expected: 0x" << std::hex << (datagram_count_ + datagram_count_offset_) << std::dec << std::endl;                             

		//reset counter to resync again
		datagram_count_ = 0; 
		
		//count as discarded message
		discarded_packets_ ++;		
		
		//return error
		return -1; 		
	}

	//At this point, port, sync and counter are checked, so we print that !
	//std::cout << "Triple check success: sender port, sync word and packet counter. " << std::endl; 
	
	//DEBUGGING: display raw packets
//	std::cout << "datagram_length_: " << datagram_length_ << std::endl;
//	std::cout << std::hex;
//	for (unsigned int ii=0; ii<datagram_length_; ii++ )
//	{
//		std::cout << std::setfill('0') << std::setw(2) << (unsigned int)raw_buffer_[ii];
//	}
//	std::cout << std::dec << std::endl;
	
	//copy raw_buffer_ to the apropiate location of raw_message_, without sync_word_ (remove first 4 bytes)
	memcpy( &raw_message_[datagram_count_*(datagram_length_-4)], &raw_buffer_[4], datagram_length_-4 );
	
	//increment packet counter
	datagram_count_ ++;
			
	//if first packet 
	if ( datagram_count_ == 1 )
	{
		//return first packet, useful for time stamping
		return FIRST_PACKET; 		
	}
	
	//check if stack is complete to form the entire message
	if ( datagram_count_ == datagram_per_msg_ ) 
	{
		//reset counter
		datagram_count_ = 0;
		
		//increment total messages
		total_messages_ ++;
		
		//Display message loss statistics:
		//std::cout << "discarded packets/correct messages: " << discarded_packets_ << "/" << total_messages_ << std::endl;
		
		//return, message complete
		return MESSAGE_COMPLETE;
	}

	//if neither first nor last packet, return still stacking packets
	return STACKING_PACKETS; 
}
