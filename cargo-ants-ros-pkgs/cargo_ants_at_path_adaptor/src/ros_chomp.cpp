/* ROS-CHOMP.
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
 * \file ros_chomp.cpp
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

using namespace std;
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef cargo_ants_msgs::ObstacleMap ObstacleMap;
typedef cargo_ants_msgs::ReferenceTrajectory Trajectory;
typedef cargo_ants_msgs::ReferenceTrajectoryPoint TrajectoryPoint;
Trajectory trajectory;
ObstacleMap obstacleMap;
Vector qs(2), qe(2), xi;
//display trajectory in rviz
visualization_msgs::MarkerArray marker_array_msg;

Trajectory generateTrajectory(Vector const &xi)
{
	Trajectory trajectory;
	trajectory.dt = 1.0;
	Vector xi_copy(xi.size()+qs.size()+qe.size());
	xi_copy<<qs,xi,qe; //copy xi and add starting point and end point
	vector <TrajectoryPoint> points(xi_copy.size() / 2);
	for (size_t ii = 0; ii < xi_copy.size() / 2; ii++){
		points[ii].xx = xi_copy[ii*2];
		points[ii].yy = xi_copy[ii*2+1];
		//finite backward difference,mind the indicies!
		if (ii>0){
			points[ii].xd = (points[ii].xx-points[ii-1].xx)/trajectory.dt;
			points[ii].yd = (points[ii].yy-points[ii-1].yy)/trajectory.dt;
			//heading
			points[ii].th = atan2(points[ii].yy-points[ii-1].yy, points[ii].xx-points[ii-1].xx);
			
		}
		if (ii>1){
			points[ii].xdd = (points[ii].xd-points[ii-1].xd)/trajectory.dt;
			points[ii].ydd = (points[ii].yd-points[ii-1].yd)/trajectory.dt;
			points[ii].thd = (points[ii].th-points[ii-1].th)/trajectory.dt;			
		}
		if (ii>2){
			points[ii].thd = (points[ii].thd-points[ii-1].thd)/trajectory.dt;			
		}
	}
	trajectory.points.insert(trajectory.points.end(), points.begin(), points.end());
	return trajectory;
}

visualization_msgs::MarkerArray generateMarkers(Trajectory t){
    //display trajectory in rviz
    visualization_msgs::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(t.points.size());//final->width * final->height);
	for ( int i = 0; i < t.points.size(); i++) {
	    marker_array_msg.markers[i].header.frame_id = "at_base_link";
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
	int n = goals.size();
        qs << goals[0].gx, goals[0].gy;
        qe << goals[n].gx, goals[n].gy;
        xi[0] = goals[0].gx;
	for (size_t ii = 0; ii < goals.size() + 1; ii++) {
		xi[ii * 2 + 1] = goals[ii].gy;
                xi[ii * 2 + 2] = goals[ii + 1].gx;
	}
}

void obstaclesCallback(const cargo_ants_msgs::ObstacleMap::ConstPtr &msg)
{
	//ROS_INFO("OBSTACLES RECEIVED BY PATH ADAPTOR! " );
	obstacleMap.obstacles = msg->obstacles;
}

void loadObstacles(Matrix &obs) {
	Matrix mapObs = Eigen::MatrixXd::Zero(3,obstacleMap.obstacles.size()); //obstaclesInMap
	size_t numObs = 0;
	for (size_t ii = 0; ii < obstacleMap.obstacles.size() ; ++ii) {
	Vector cr(2), dc(4); 
	Matrix dcr(2,4); //corners
	cr << 0, 0; //centroid (x,y) wrt at_base_link
	for (size_t pii = 0; pii < 4 ; ++pii) {
			dcr.col(pii) << obstacleMap.obstacles[ii].polylines[0].points[pii].px,
		 								obstacleMap.obstacles[ii].polylines[0].points[pii].py;			 								
		 	cr << cr(0) + obstacleMap.obstacles[ii].polylines[0].points[pii].px, cr(1) + obstacleMap.obstacles[ii].polylines[0].points[pii].py;							
		}
	cr = 0.25 * cr;
	Matrix Cr(2,4);
	Cr << cr(0), cr(0), cr(0), cr(0),
	             cr(1), cr(1), cr(1), cr(1);
	dcr = dcr - Cr; // distances between centroid and corners
	//cout << "centroid:" << cr << endl;
	dc << dcr.block(0, 0, 2, 1).norm(),  
	     dcr.block(0, 1, 2, 1).norm(),
	     dcr.block(0, 2, 2, 1).norm(),
	     dcr.block(0, 3, 2, 1).norm(); 
	double rr = dc.maxCoeff();      
	//cout << "radius:" << rr << endl;  
	
	//Filter out some obstacles
	if(rr > 0.5 && cr(0) > 7.){ 
		mapObs.col(numObs) << cr(0), cr(1),  rr;
		numObs++; 		
	    }
	}
	obs.resize(3,numObs);
	obs = mapObs.block(0,0,3,numObs);
	/*
	cout << "Original Obs. Matrix of size 3 x "<< mapObs.cols()<<  endl;	
	cout <<  mapObs << endl << endl;			
	cout << "Filtered Obs. Matrix of size 3 x "<< obs.cols()<<  endl;	
	cout <<  obs << endl << endl;	
	*/
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_adaptor");
	ros::NodeHandle node;
	ros::Subscriber path_sub = node.subscribe("path_planner", 1000, pathPlannerCallback);
	ros::Subscriber obs_sub = node.subscribe("obstacles", 1000, obstaclesCallback);
	ros::Publisher trajectory_pub = node.advertise<Trajectory> ( "/trajectory", 10);
	//display trajectory in rviz
    ros::Publisher pub_marker = node.advertise<visualization_msgs::MarkerArray>("chompTrajectory", 100);
	
	ros::Rate loop_rate (10);	
	ROS_INFO("AT_Path_Adaptor Started!");
	while ( ros::ok() ){
		ros::spinOnce();

		Matrix obs(3,obstacleMap.obstacles.size());
		for (size_t ii = 0; ii < obstacleMap.obstacles.size() ; ++ii) {
			Vector cr(2), dc(4); 
			Matrix dcr(2,4);
			cr << 0, 0;
			for (size_t pii = 0; pii < 4 ; ++pii) {
				dcr.col(pii) << obstacleMap.obstacles[ii].polylines[0].points[pii].px,
			 								obstacleMap.obstacles[ii].polylines[0].points[pii].py;			 								
			 	cr << cr(0) + obstacleMap.obstacles[ii].polylines[0].points[pii].px, cr(1) + obstacleMap.obstacles[ii].polylines[0].points[pii].py;							
			}
			cr = 0.25 * cr;  //1/4 sides
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
		}
		//	Matrix obs;				
        loadObstacles(obs);	// Load obstacles        
		chomp::generatePath(qs, qe, xi, obs);
		trajectory = generateTrajectory(xi);
		marker_array_msg = generateMarkers(trajectory);
		trajectory_pub.publish(trajectory);  //publish CHOMP's trajectory with our custom defined msg
		pub_marker.publish(marker_array_msg); //publish array of poses for visualizing CHOMP's trajectory in Rviz
		loop_rate.sleep();
    }
	return 0;
	// VectorXd qs(2);  //Start goal  coordinates (x,y)
	// VectorXd qe(2);  //End goal  coordinates (x,y)
	// VectorXd xi;     //Trajectory points (x0,y0,x1,y1,....)
	// MatrixXd obs;    //obstacles |x0,y0,R0;x1,y1,R1...|
	//
	// qs << 0, 0;
	// qe << 3, 5;
	//
	//
	// generatePath(qs, qe, xi, obs);
	// cout << xi << std::endl;
}
