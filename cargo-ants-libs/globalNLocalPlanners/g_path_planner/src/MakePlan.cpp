#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include <cargo_ants_msgs/Path.h>       //received from path_planner
#include <cargo_ants_msgs/Goal.h>       // member of Path msg
#include <cargo_ants_msgs/ReferenceTrajectory.h>
#include <cargo_ants_msgs/ReferenceTrajectoryPoint.h>
#include <cargo_ants_msgs/Path.h>
#include <cargo_ants_msgs/Goal.h>
#include <cargo_ants_msgs/ObstacleMap.h>
#include <cargo_ants_msgs/Obstacle.h>
#include <cargo_ants_msgs/Polyline.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

using std::string;
using namespace std;
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef cargo_ants_msgs::Path Path;
typedef cargo_ants_msgs::Goal PathPoint;
Path GlobalPath;
bool PathIsPublished = false;


//display trajectory in rviz
visualization_msgs::MarkerArray marker_array_msg;

double g_GoalTolerance = 0.5;
string g_WorldFrame = "map";
void fillPathRequest(nav_msgs::GetPlan::Request &req);
void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);


void fillPathRequest(nav_msgs::GetPlan::Request &request)
{
	// TODO: send start/goal as an external defined parameter, 
	//       now they are hardcoded here, 
	// (26.735, -2.376)  (10.5, -0.5)
	request.start.header.frame_id = g_WorldFrame;
	request.start.pose.position.x = 0.0;
	request.start.pose.position.y = 0.0;
	tf::Quaternion qs = tf::createQuaternionFromYaw(0);  	    
	request.start.pose.orientation.x = qs.getX();
	request.start.pose.orientation.y = qs.getY();
	request.start.pose.orientation.z = qs.getZ();
	request.start.pose.orientation.w = qs.getW();

	request.goal.header.frame_id = g_WorldFrame;
	request.goal.pose.position.x = 26.73; 
	request.goal.pose.position.y = -2.37;
	tf::Quaternion qg = tf::createQuaternionFromYaw(0);  	    
	request.goal.pose.orientation.x = qg.getX();
	request.goal.pose.orientation.y = qg.getY();
	request.goal.pose.orientation.z = qg.getZ();
	request.goal.pose.orientation.w = qg.getW();	
	
	request.goal.pose.orientation.w = 1.0;

	request.tolerance = g_GoalTolerance;
}




visualization_msgs::MarkerArray generateMarkers(vector <PathPoint> goals){
    //display trajectory in rviz
    visualization_msgs::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(goals.size());//final->width * final->height);
	for ( int i = 0; i < goals.size(); i++) {
	    marker_array_msg.markers[i].header.frame_id = "map";
	    marker_array_msg.markers[i].header.stamp = ros::Time();
	    marker_array_msg.markers[i].ns = "my_namespace";
	    marker_array_msg.markers[i].id = i;
	    marker_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
	    marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
	    marker_array_msg.markers[i].pose.position.x = goals[i].gx;
	    marker_array_msg.markers[i].pose.position.y = goals[i].gy;
	    marker_array_msg.markers[i].pose.position.z = 0;
	    tf::Quaternion qt = tf::createQuaternionFromYaw(goals[i].gth);  	    
	    marker_array_msg.markers[i].pose.orientation.x = qt.getX();
	    marker_array_msg.markers[i].pose.orientation.y = qt.getY();
	    marker_array_msg.markers[i].pose.orientation.z = qt.getZ();
	    marker_array_msg.markers[i].pose.orientation.w = qt.getW();
	    marker_array_msg.markers[i].scale.x = 0.1;
	    marker_array_msg.markers[i].scale.y = 0.1;
	    marker_array_msg.markers[i].scale.z = 0.1;
	    marker_array_msg.markers[i].color.a = 1.0;
	    marker_array_msg.markers[i].color.g = 0.0;
	    marker_array_msg.markers[i].color.b = 0.0; 
	    if (i == 0)
	    {
	        marker_array_msg.markers[i].color.r = 0.1;
	    }
	    else
	    {
	        marker_array_msg.markers[i].color.r = i * 0.15;
	    }
	    
	}
	return marker_array_msg;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "make_plan_node");
	ros::NodeHandle nh;
	ros::Publisher path_pub = nh.advertise<Path> ( "/path_planner", 10);
	//display trajectory in rviz
    ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>("GlobalPath", 100);
	ros::Rate loop_rate (10);
	// Init service query for make plan
	string service_name = "move_base/NavfnROS/make_plan"; //for NavFn
	//string service_name = "move_base/make_plan";
	
	while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
		ROS_INFO("Waiting for service move_base/make_plan to become available");
	}

	ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
	if (!serviceClient) {
		ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());
		return -1;
	}

	nav_msgs::GetPlan srv;
	fillPathRequest(srv.request);

	if (!serviceClient) {
		ROS_FATAL("Persistent service connection to %s failed", serviceClient.getService().c_str());
		return -1;
	}

	// Perform the actual path planner call
	if (serviceClient.call(srv)) {
			if (!srv.response.plan.poses.empty()) {				
			  while ( ros::ok() ){
			        ros::spinOnce();
					unsigned int numPathPoints =  srv.response.plan.poses.size();
					vector <PathPoint> goals(numPathPoints);
					unsigned int ii=0;
					forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
						//ROS_INFO("x = %f, y = %f ",  p.pose.position.x, p.pose.position.y);
						double yaw_angle = tf::getYaw(p.pose.orientation);
						/*ROS_INFO("x = %f, y = %f, qx = %f, qy = %f, qz = %f, qw = %f", 
						 p.pose.position.x, p.pose.position.y, p.pose.orientation.y, 
						 p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
						double yaw_angle = tf::getYaw(p.pose.orientation);*/
						goals[ii].gx = p.pose.position.x;
						goals[ii].gy = p.pose.position.y;
						goals[ii].gth = yaw_angle;				
						ii++;
					}
					///
					// Reduce number of points
					float percentPts = 0.05; //TODO: can be a parameter or better filter them out by path length
					unsigned int numPPts = floor(numPathPoints * percentPts);
					vector <PathPoint> goalsToSend(numPPts);
					int incPts = floor(numPathPoints / numPPts);
					unsigned int j=0;
					for(size_t i = 0; i < numPathPoints; i+=incPts) {
						if(j<numPPts) goalsToSend[j] = goals[i];
						j++;
				    }
				    goalsToSend[numPPts-1] = goals[numPathPoints-1];
					
					///
					GlobalPath.goals.insert(GlobalPath.goals.end(), goalsToSend.begin(), goalsToSend.end());
					marker_array_msg = generateMarkers(goals);
					//if (!PathIsPublished){ //publish path once
						PathIsPublished = true;
					    path_pub.publish(GlobalPath);  //publish Global Path Planner's path with our custom defined msg
					    ROS_INFO("....................... publishing path! .....................................");
					    
					//}
					pub_marker.publish(marker_array_msg); //publish array of poses for visualizing Global Path Planner's path in Rviz
			   }
			}
			else {
				ROS_WARN("Got empty plan");
			}
		}
		else {
			ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());
		}
    loop_rate.sleep();

	return 0;
}



