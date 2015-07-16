
//ros dependencies
#include "lib_wrapper.h"

//node main
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "iri_laser_corners_node");
    
    //create ros wrapper object
    LibWrapper myLibWrapper;
    
    //set node loop rate
    ros::Rate nodeRate(myLibWrapper.loop_rate_);    
    
    //node loop 
    while ( ros::ok() )
    {
        //execute pending callbacks (if subscribed to something)
        //ros::spinOnce(); 
        
        if ( myLibWrapper.newData() ) // it has sense if waiting for data ...
        {
            //do things
            myLibWrapper.process();

            //do things
            myLibWrapper.publish();
        }

        //relax to fit output rate
        nodeRate.sleep();            
    }
        
    //exit program
    return 0;
}