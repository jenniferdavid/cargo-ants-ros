
//this package dependencies
#include "at_drive_sim_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "at_drive_node");
      
      //create ros wrapper object
      AtDriveNode at;
      
      //set node loop rate
      ros::Rate looprate(at.getRate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 

            //do things
            at.process();
            
            //publish
            at.publish();
                        
            //relax to fit output rate
            looprate.sleep();            
      }
            
      //exit program
      return 0;
}