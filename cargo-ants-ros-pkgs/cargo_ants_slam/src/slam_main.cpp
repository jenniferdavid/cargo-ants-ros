
//this package dependencies
#include "slam_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "slam");
      
      //create ros wrapper object
      SlamNode slam;
      
      //set node loop rate
      ros::Rate looprate(slam.getRate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 

            //do things
            slam.process();
            
            //publish
            slam.publish();
                        
            //relax to fit output rate
            looprate.sleep();            
      }
            
      //exit program
      return 0;
}
