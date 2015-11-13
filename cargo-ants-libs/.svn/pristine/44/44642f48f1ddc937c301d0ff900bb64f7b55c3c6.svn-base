/* ROS-CHOMP.
 *
 * Copyright (C) 2015 Jafar Qutteineh. All rights reserved.
 * License (3-Cluase BSD): https://github.com/j3sq/ROS-CHOMP/blob/master/LICENSE
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

using namespace std;
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef cargo_ants_msgs::ReferenceTrajectory Trajectory;
typedef cargo_ants_msgs::ReferenceTrajectoryPoint TrajectoryPoint;
Trajectory trajectory;

Trajectory generateTrajectory(Vector const &xi)
{
	Trajectory trajectory;
	trajectory.dt = 1.0;
	vector <TrajectoryPoint> points(xi.size() / 2);
	for (size_t ii = 0; ii < xi.size() / 2; ii++){
		points[ii].xx = xi[ii*2];
		points[ii].yy = xi[ii*2+1];
		//finite backward difference,mind the indicies!
		if (ii>0){
			points[ii].xd = (points[ii].xx-points[ii-1].xx)/trajectory.dt;
			points[ii].yd = (points[ii].yy-points[ii-1].yy)/trajectory.dt;
		}
		if (ii>1){
			points[ii].xdd = (points[ii].xd-points[ii-1].xd)/trajectory.dt;
			points[ii].ydd = (points[ii].yd-points[ii-1].yd)/trajectory.dt;
		}
		//ds
	}
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
	Vector qs(2), qe(2), xi;
	Matrix obs;
	for (size_t ii = 0; ii < goals.size() - 1; ii++) {
		qs << goals[ii].gx, goals[ii].gy;
		qe << goals[ii + 1].gx, goals[ii + 1].gy;
		chomp::generatePath(qs, qe, xi, obs);
		trajectory = generateTrajectory(xi);

	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_adaptor");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("path_planner", 1000, pathPlannerCallback);
	ros::Publisher trajectory_pub = node.advertise<Trajectory> ( "/trajectory", 10);
	ros::Rate loop_rate (10);
	ROS_INFO("path_adaptor Started!");
	while ( ros::ok() ){
	ros::spinOnce();
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
