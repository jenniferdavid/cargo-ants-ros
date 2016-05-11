/* Path_adaptor_node.
 * 
 * Copyright (C) 2016, 2015 Rafael Valencia, Jennifer David. All rights reserved.
 * Based on code by 
 * 
 * This code uses and is based on code from:
 *   
 *   Copyright (C) 2015 Jafar Qutteineh. All rights reserved.
 *   License (3-Cluase BSD): https://github.com/j3sq/ROS-CHOMP/blob/master/LICENSE
 *    
 *   Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *   License (3-Clause BSD) : https://github.com/poftwaresatent/trychomp
 *  
 * 
 * **
 * \file path_adaptor_node.cpp
 *
 * ROS support for chomp path adaptor for Cargo-ANTS project http://cargo-ants.eu/
 *
 *
 */
#include <iostream>
#include <Eigen/Dense>
#include "chomp.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cargo_ants_msgs/Path.h>       //received from path_planner
#include <cargo_ants_msgs/Goal.h>       // member of Path msg
#include <cargo_ants_msgs/ReferenceTrajectory.h>
#include <cargo_ants_msgs/ReferenceTrajectoryPoint.h>
#include <cargo_ants_msgs/ObstacleMap.h>
#include <cargo_ants_msgs/Obstacle.h>
#include <cargo_ants_msgs/Polyline.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef cargo_ants_msgs::ObstacleMap ObstacleMap;
typedef cargo_ants_msgs::ReferenceTrajectory Trajectory;
typedef cargo_ants_msgs::ReferenceTrajectoryPoint TrajectoryPoint;
Trajectory trajectory;
ObstacleMap obstacleMap;
double agvLocX, agvLocY;
Vector qs(2), qe(2), xi, xs(2), xe(2);
Vector gqs(2), gqe(2), gxi; //global start, global goal, global path
unsigned int gnq;             // number of q stacked into gxi
unsigned int gxidim;    // dimension of trajectory, gxidim = gnq * cdim
unsigned int nq;             // number of q stacked into xi
unsigned int cdim = 2;            // dimension of config space
unsigned int xidim;    // dimension of trajectory, xidim = nq * cdim
 

bool ReceivedPath = false;

//display trajectory in rviz
visualization_msgs::MarkerArray marker_array_msg;

Trajectory generateTrajectory(Vector const &xi)
{
	Trajectory trajectory;
	trajectory.dt = 1.0;
	Vector xi_copy(xi.size()+qs.size()+qe.size());
	xi_copy<<qs,xi,qe; //copy xi and add starting point and end point

	size_t N = xi_copy.size() / 2;
	vector <TrajectoryPoint> points(N);

	for (size_t ii = 0; ii < N; ii++){
		points[ii].xx = xi_copy[ii*2];
		points[ii].yy = xi_copy[ii*2+1];
	}
	for (size_t ii = 0; ii < N-1; ii++){
		    //first derivative of x-y 
			points[ii].xd = (points[ii+1].xx-points[ii].xx)/trajectory.dt;
			points[ii].yd = (points[ii+1].yy-points[ii].yy)/trajectory.dt;
			//heading
			points[ii].th = atan2(points[ii+1].yy-points[ii].yy, points[ii+1].xx-points[ii].xx);
	}
	for (size_t ii = 0; ii < N-1; ii++){
		    //second derivative of x-y 
			points[ii].xdd = (points[ii+1].xd-points[ii].xd)/trajectory.dt;
			points[ii].ydd = (points[ii+1].yd-points[ii].yd)/trajectory.dt;
			//first derivative's heading 
			points[ii].thd = atan2(points[ii+1].yd-points[ii].yd, points[ii+1].xd-points[ii].xd) / trajectory.dt;
	}	
	for (size_t ii = 0; ii < N-1; ii++){
			//second derivative's heading 
			points[ii].thdd = atan2(points[ii+1].ydd-points[ii].ydd, points[ii+1].xdd-points[ii].xdd) / trajectory.dt;
	}
	points[N-1].xx  = xi_copy[(N-1)*2];
	points[N-1].yy  = xi_copy[(N-1)*2+1];
	points[N-1].th = points[N-2].th;
	points[N-1].xd =  points[N-2].xd;
	points[N-1].yd =  points[N-2].yd; 
	points[N-1].thd = points[N-2].thd;
    points[N-1].xdd =  points[N-2].xdd; 
	points[N-1].ydd =  points[N-2].ydd; 
	points[N-1].thdd = points[N-2].thdd;
	 
	trajectory.points.insert(trajectory.points.end(), points.begin(), points.end());
	return trajectory;
}

visualization_msgs::MarkerArray generateMarkers(Trajectory t){
    //display trajectory in rviz
    visualization_msgs::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(t.points.size());//final->width * final->height);
	for ( int i = 0; i < t.points.size(); i++) {
	    marker_array_msg.markers[i].header.frame_id = "odom";
	    marker_array_msg.markers[i].header.stamp = ros::Time();
	    marker_array_msg.markers[i].ns = "my_namespace";
	    marker_array_msg.markers[i].id = i;
	    marker_array_msg.markers[i].type = visualization_msgs::Marker::ARROW;
	    marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
	    marker_array_msg.markers[i].pose.position.x = t.points[i].xx;
	    marker_array_msg.markers[i].pose.position.y = t.points[i].yy;
	    marker_array_msg.markers[i].pose.position.z = 0;
	    tf::Quaternion qt = tf::createQuaternionFromYaw(t.points[i].th);   	    
	    marker_array_msg.markers[i].pose.orientation.x = qt.getX();
	    marker_array_msg.markers[i].pose.orientation.y = qt.getY();
	    marker_array_msg.markers[i].pose.orientation.z = qt.getZ();
	    marker_array_msg.markers[i].pose.orientation.w = qt.getW();
	    marker_array_msg.markers[i].scale.x = 1;
	    marker_array_msg.markers[i].scale.y = 0.1;
	    marker_array_msg.markers[i].scale.z = 0.1;
	    marker_array_msg.markers[i].color.a = 1.0;
	    marker_array_msg.markers[i].color.r = 0.0;
	    if (i == 0)
	    {
	        marker_array_msg.markers[i].color.g = 0.1;
	    }
	    else
	    {
	        marker_array_msg.markers[i].color.g = i * 0.15;
	    }
	    marker_array_msg.markers[i].color.b = 0.0; 
	}
	return marker_array_msg;
}

void pathPlannerCallback(const cargo_ants_msgs::Path::ConstPtr &msg)
{
	vector <cargo_ants_msgs::Goal> goals = msg->goals;
	if (goals.size() < 2) {
		ROS_INFO("At least 2 goal points are required, received %lu", goals.size());
		return;
	}
	if(!ReceivedPath){		
					
		gnq = goals.size();             // number of q stacked into xi
        
        cout << " PATH ADAPTOR RECEIVED THE FOLLOWING GLOBAL PATH: " << endl;	
		for (size_t ii = 0; ii <gnq; ii++) {  
		 cout << " GoalX: " << goals[ii].gx << "  GoalY: " << goals[ii].gy << endl;		 
		}		
		
        gxidim = (gnq-2) * cdim ; // number of intermediate points, without start and goal 
        gxi = Vector::Zero(gxidim);  
        size_t jj = 0;
		for (size_t ii = 1; ii < gnq-1; ii++) {  
		  gxi.block(cdim * jj, 0, cdim, 1) << goals[ii].gx, goals[ii].gy; 
		  jj++;	 
		}		
		
		gqs << goals[0].gx, goals[0].gy;
		gqe << goals[gnq-1].gx, goals[gnq-1].gy;        
        ReceivedPath = true;		
	}
}

void obstaclesCallback(const cargo_ants_msgs::ObstacleMap::ConstPtr &msg)
{
	//ROS_INFO("OBSTACLES RECEIVED BY PATH ADAPTOR! " );
	obstacleMap.obstacles = msg->obstacles;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_adaptor");
	ros::NodeHandle node;
	ros::Subscriber path_sub = node.subscribe("/path_planner", 1000, pathPlannerCallback);
	ros::Subscriber obs_sub = node.subscribe("obstacles", 1000, obstaclesCallback);
	ros::Publisher trajectory_pub = node.advertise<Trajectory> ( "/trajectory", 10);
	//display trajectory in rviz
    ros::Publisher pub_marker = node.advertise<visualization_msgs::MarkerArray>("chompTrajectory", 100);

	ros::Rate loop_rate (10);	
	ROS_INFO("AGV_Path_Adaptor Started!");
	while ( ros::ok() ){
		ros::spinOnce();
		Matrix obs(3,obstacleMap.obstacles.size());
		/*for (size_t ii = 0; ii < obstacleMap.obstacles.size() ; ++ii) {			
			Vector cr(2), dc(4); 
			Matrix dcr(2,4);
			cr << 0, 0;
			for (size_t pii = 0; pii < 4 ; ++pii) {
				dcr.col(pii) << obstacleMap.obstacles[ii].polylines[0].points[pii].px,
			 								obstacleMap.obstacles[ii].polylines[0].points[pii].py;			 								
			 	cr << cr(0) + obstacleMap.obstacles[ii].polylines[0].points[pii].px, cr(1) + obstacleMap.obstacles[ii].polylines[0].points[pii].py;							
			}
			cr = 0.25 * cr;  
			Matrix Cr(2,4);
			Cr << cr(0), cr(0), cr(0), cr(0),
			             cr(1), cr(1), cr(1), cr(1);
			dcr = dcr - Cr;  //radius 
			//cout << "centroid:" << cr << endl;
			dc << dcr.block(0, 0, 2, 1).norm(),  
			  dcr.block(0, 1, 2, 1).norm(),   //calculating each distance from each of the corners to the centroid
			     dcr.block(0, 2, 2, 1).norm(),
			     dcr.block(0, 3, 2, 1).norm(); 
			double rr = dc.maxCoeff(); //maximum distance from the centroid     
			//cout << "radius:" << rr << endl;  
			obs.col(ii) << cr(0), cr(1),  rr;																								
		}*/			
        
        /*
        TODO:
			1. Load obstacles from obstacleMap.obstacles in "obs" matrix
			2. Select the chunk of the global path to adapt, 
			   i.e. fill "xi, qs, qe" vectors with the path to adapt 
			3. uncomment chomp and send the local path         
		*/
		//chomp::generatePath(qs, qe, xi, obs);
		// TODO: for testing purposes we're bypassing chomp and send out the global path to the vehicle (we need to remove this)
		qs = gqs; qe = gqe; xi = gxi; //remove this when chomp is called
		
		trajectory = generateTrajectory(xi);
		marker_array_msg = generateMarkers(trajectory);
		trajectory_pub.publish(trajectory);  //publish CHOMP's trajectory with our custom defined msg
		pub_marker.publish(marker_array_msg); //publish array of poses for visualizing CHOMP's trajectory in Rviz
		loop_rate.sleep();
    }
	return 0;
 
}
