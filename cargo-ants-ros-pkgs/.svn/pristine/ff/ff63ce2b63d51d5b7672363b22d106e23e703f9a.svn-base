/* ROS-CHOMP.
 * 
 * Copyright (C) 2015 Rafael Valencia, Jennifer David. All rights reserved.
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

using namespace std;
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef cargo_ants_msgs::ObstacleMap ObstacleMap;
typedef cargo_ants_msgs::ReferenceTrajectory Trajectory;
typedef cargo_ants_msgs::ReferenceTrajectoryPoint TrajectoryPoint;
Trajectory trajectory;
ObstacleMap obstacleMap;
Vector qs(2), qe(2), xi;

Trajectory generateTrajectory(Vector const &xi)
{
	Trajectory trajectory;
	trajectory.dt = 0.1;
	Vector xi_copy(xi.size()+qs.size()+qe.size());
	xi_copy<<qs,xi,qe; //copy xi and add starting point and end point
	vector <TrajectoryPoint> points(xi_copy.size() / 2);
	for (size_t ii = 0; ii < xi_copy.size() / 2; ii++){
		points[ii].xx = xi_copy[ii*2];
		points[ii].yy = xi_copy[ii*2+1];
		//finite backward difference,mind the indicies!
		if (ii>0){
			points[ii-1].xd = (points[ii].xx-points[ii-1].xx)/trajectory.dt;
			points[ii-1].yd = (points[ii].yy-points[ii-1].yy)/trajectory.dt;
			//heading
			points[ii-1].th = atan2(points[ii].yy-points[ii-1].yy, points[ii].xx-points[ii-1].xx);
			
		}
		if (ii>1){
			points[ii-2].xdd = (points[ii].xd-points[ii-1].xd)/trajectory.dt;
			points[ii-2].ydd = (points[ii].yd-points[ii-1].yd)/trajectory.dt;		
		}
		if (ii>2){
			points[ii-2].thd = (points[ii].th-points[ii-1].th)/trajectory.dt;			
		}
		if (ii>3){
			points[ii-3].thdd = (points[ii].thd-points[ii-1].thd)/trajectory.dt;			
		}		
	}
	trajectory.start = ros::Time::now();
	trajectory.points.insert(trajectory.points.end(), points.begin(), points.end());
	return trajectory;
}
void pathPlannerCallback(const cargo_ants_msgs::Path::ConstPtr &msg)
{
	vector <cargo_ants_msgs::Goal> goals = msg->goals;
	if (goals.size() < 2) {
		ROS_INFO("At least 2 goal points are required, received %lu", goals.size());
		return;
	}
	for (size_t ii = 0; ii < goals.size() - 1; ii++) {
		qs << goals[ii].gx, goals[ii].gy;
		qe << goals[ii + 1].gx, goals[ii + 1].gy;
	}
}

void obstaclesCallback(const cargo_ants_msgs::ObstacleMap::ConstPtr &msg)
{
	obstacleMap.obstacles = msg->obstacles;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "agv_path_adaptor");
	ros::NodeHandle node;
	ros::Subscriber path_sub = node.subscribe("path_planner", 1000, pathPlannerCallback);
	ros::Subscriber obs_sub = node.subscribe("obstacles", 1000, obstaclesCallback);
	ros::Publisher trajectory_pub = node.advertise<Trajectory> ( "/trajectory", 10);
	ros::Rate loop_rate (0.2);
	ROS_INFO("AGV_path_adaptor Started!");
	while ( ros::ok() ){
		ros::spinOnce();
		Matrix obs(3,obstacleMap.obstacles.size());
		for (size_t ii = 0; ii < obstacleMap.obstacles.size() ; ++ii) {
			obs.col(ii) << obstacleMap.obstacles[ii].origin.ox,
			 								obstacleMap.obstacles[ii].origin.oy,
											obstacleMap.obstacles[ii].origin.oth;
		}
		chomp::generatePath(qs, qe, xi, obs);
		trajectory = generateTrajectory(xi);
		trajectory_pub.publish(trajectory);
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
