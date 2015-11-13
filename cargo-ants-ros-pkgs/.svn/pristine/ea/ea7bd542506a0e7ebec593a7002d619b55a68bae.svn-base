
//this package dependencies
#include "local_map_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "local_mapping");
      
      //create ros wrapper object
      LocalMapNode lmap;
      
      //set node loop rate
      ros::Rate looprate(lmap.getRate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 

            //do things
            lmap.process();
            
            //publish
            lmap.publish();
                        
            //relax to fit output rate
            looprate.sleep();            
      }
            
      //exit program
      return 0;
}