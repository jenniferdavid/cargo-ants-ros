#include "lib_wrapper.h"

LibWrapper::LibWrapper()
{    
    //init our data 
    count_ = 173; 
    
    // set rate in hz
    loop_rate_ = 5.0; // hz

    //init publishers
    publisher_ = nh_.advertise<std_msgs::String>("debug", 100);
    
    //link callbacks to subscribers
    // to do 
}

LibWrapper::~LibWrapper()
{
    // do things if necessary
}

bool LibWrapper::newData()
{
    return true; //could be more complicated ...
}

void LibWrapper::process()
{
    //do process
    count_ ++;
    
    //fill message
    out_msg_.data = "Hello Cargo-ANTs Team !";
    
}
            
void LibWrapper::publish()
{
    publisher_.publish(out_msg_);
}
