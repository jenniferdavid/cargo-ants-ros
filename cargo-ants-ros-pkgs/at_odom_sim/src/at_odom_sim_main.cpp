
//this package dependencies
#include "at_odom_sim_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "at_odom_sim_node");
      
      //create ros wrapper object
      AtOdomSimNode at_odom;
      
      //set node loop rate
      ros::Rate looprate(at_odom.getRate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 

            //publish
            at_odom.publish();
                        
            //relax to fit output rate
            looprate.sleep();            
      }
            
      //exit program
      return 0;
}