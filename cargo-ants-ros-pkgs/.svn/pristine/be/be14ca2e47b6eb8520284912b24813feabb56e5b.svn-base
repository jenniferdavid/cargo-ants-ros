
#include "ros/ros.h"
#include "ros/time.h"

#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_state_estimator_node");   
    
    geometry_msgs::PoseStamped vehicle_pose_msg;
    
    ros::Publisher vehicle_state_publisher;
    

    
    ros::NodeHandle nh( ros::this_node::getName() );
    
    ros::Rate loop_rate(20);
    
    vehicle_state_publisher = nh.advertise<geometry_msgs::PoseStamped>("poseStamped", 100);
    
    double x = 0.0;
    
    while ( ros::ok() )
    {

        
        vehicle_pose_msg.header.seq ++;
        vehicle_pose_msg.header.stamp = ros::Time::now();
        vehicle_pose_msg.header.frame_id = "site";
        
        x++;
        
        vehicle_pose_msg.pose.position.x = x;
        vehicle_pose_msg.pose.position.y = 2.0;
        vehicle_pose_msg.pose.position.z = 3.0;

        vehicle_pose_msg.pose.orientation.x = 1.0;        
        vehicle_pose_msg.pose.orientation.y = 0.0;        
        vehicle_pose_msg.pose.orientation.z = 0.0;        
        vehicle_pose_msg.pose.orientation.w = 0.0;       
        
        vehicle_state_publisher.publish(vehicle_pose_msg);
        
        ros::spinOnce();
        
        loop_rate.sleep();
        
    }
}

