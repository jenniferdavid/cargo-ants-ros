/* Position Feedback Controller for Turtlebot
 *
 * Copyright (C) 2015 Jafar Qutteineh. All rights reserved.
 * License (3-Cluase BSD): TODO: Add github link
 *
 * This code uses and is based on code from:
 *   Project: trychomp https://github.com/poftwaresatent/trychomp
 *   Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *   License (3-Clause BSD) : https://github.com/poftwaresatent/trychomp
 * **
 * \file chomp.cpp
 *
 * CHOMP for point vehicles (x,y) moving holonomously in the plane. It will
 * plan a trajectory (xi) connecting start point (qs) to end point (qe) while
 * avoiding obstacles (obs)
 */

#include <stdlib.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cargo_ants_msgs/Goal.h>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/DeleteModel.h"
#include <gazebo_msgs/SpawnModel.h>
#include <cargo_ants_msgs/ReferenceTrajectory.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "diff_drive_robot.hpp"
#include <math.h>

#define PI 3.14159265359
#define CONTROL_LOOP_RATE 10
#define VEHICLE_NAME  "mobile_base"
#define GOALS_COUNT 22


using namespace diff_drive_robot;

double x_odd,y_odd,theta_odd; //oddometry readings
double x_true,y_true,theta_true; //oddometry readings
std::vector<diff_drive_robot::Goal> goals;
char* marker_file_name;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  x_odd = msg->pose.pose.position.x;
  y_odd = msg->pose.pose.position.y;
  theta_odd = asin(msg->pose.pose.orientation.z) * 2.0;
}
void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  for(std::vector<std::string>::size_type ii = 0; ii != msg->name.size(); ++ii) {
    if (msg->name[ii] == VEHICLE_NAME){
      x_true = msg->pose[ii].position.x;
      y_true = msg->pose[ii].position.y;
      theta_true = asin(msg->pose[ii].orientation.z) * 2.0;
      break;
    }
  }
}

void trajectory_callback(const cargo_ants_msgs::ReferenceTrajectory::ConstPtr &msg)
{
  goals.clear();
  for (size_t ii=0; ii<msg->points.size(); ++ii){
    diff_drive_robot::Goal goal;
    goal.x = msg->points[ii].xx;
    goal.y = msg->points[ii].yy;
    goal.theta = msg->points[ii].th;
    goals.push_back(goal);
  }
}

void delete_marker(ros::NodeHandle  &node, size_t goal_idx){
  static ros::ServiceClient gazebo_delete_client = node.serviceClient<gazebo_msgs::DeleteModel> ("/gazebo/delete_model");
  static gazebo_msgs::DeleteModel delete_model_srv;
  std::ostringstream ss;
  ss<<"goal_"<<goal_idx;
  std::cout<<"Deleteing "<<ss.str()<<std::endl;
  delete_model_srv.request.model_name=ss.str();
  gazebo_delete_client.call(delete_model_srv);
}

void spawn_goals_markers(ros::NodeHandle  &node,
    std::vector<diff_drive_robot::Goal> const &goals){
    ros::ServiceClient gazebo_set_model_client = node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient gazebo_spawn_client = node.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
    gazebo_msgs::SetModelState set_model_srv;

    //read marker model sdf file
    gazebo_msgs::SpawnModel model;
    std::ifstream file(marker_file_name);
    std::string line;
     while(!file.eof()) // Parse the contents of the given urdf in a string
      {
        std::getline(file,line);
        model.request.model_xml+=line;
      }
    file.close();
    model.request.reference_frame="world";
    //spawn and place a marker for each goal
    for(int ii=0; ii< GOALS_COUNT; ii++) {
      std::ostringstream ss;
      ss<<"goal_"<<ii;
      model.request.model_name=ss.str();
      gazebo_spawn_client.call(model); //Call the service
      // hide the markers by placing them far away
      set_model_srv.request.model_state.model_name = ss.str();
      set_model_srv.request.model_state.pose.position.x = 20 + ii*0.15;
      set_model_srv.request.model_state.pose.position.y = 20 + ii*0.15;
      gazebo_set_model_client.call(set_model_srv);
      std::cout<<"Done Spawning!"<<std::endl;
    }
}
void update_goals_markers(ros::NodeHandle  &node, DiffDriveRobot &tbot){

    //build model_states message
    //The goals are updated one at a time. Trails with batch updated resulted in
    //most packets being dropped.
    ros::ServiceClient gazebo_set_model_client = node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState set_model_srv;
    static size_t ii = 0;
    ii++;
    ii %= GOALS_COUNT;
    std::ostringstream ss;
    ss<<"goal_"<<ii;
    set_model_srv.request.model_state.model_name = ss.str();
    if (ii<tbot.get_goal_idx()){
      set_model_srv.request.model_state.pose.position.x = 20 + ii*0.15;
      set_model_srv.request.model_state.pose.position.y = 20 + ii*0.15;;
    }
    else{
      set_model_srv.request.model_state.pose.position.x  = tbot.get_goal(ii).x;
      set_model_srv.request.model_state.pose.position.y = tbot.get_goal(ii).y;
    }
    gazebo_set_model_client.call(set_model_srv);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_controller");
  marker_file_name =argv[argc-1];
  std::cout<<"Path = "<<marker_file_name<<std::endl;
  ros::NodeHandle node;
	ros::Publisher twist_pub = node.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/navi", 1);
  ros::Publisher model_states_pub = node.advertise<gazebo_msgs::ModelState> ("gazebo/set_model_state", 1000);
	ros::Subscriber odom_sub = node.subscribe("odom", 1000, odom_callback);
  ros::Subscriber trajectory_sub = node.subscribe("trajectory", 1000, trajectory_callback);
  ros::Subscriber model_states_sub = node.subscribe("/gazebo/model_states",
                                              1000, model_states_callback);
  ros::Rate loop_rate(CONTROL_LOOP_RATE);
	geometry_msgs::Twist twist;
	twist.linear.x = 1.0;
	twist.angular.z = 0.0;
	ROS_INFO("pf_controller started");
  DiffDriveRobot tbot(0.5, PI/2.0, 0.5/2, PI/4.0, 1.0/CONTROL_LOOP_RATE);
  spawn_goals_markers(node,goals);
  size_t last_goal_idx = 0;
	while (ros::ok()) {
		ros::spinOnce();
    tbot.set_goals(goals);
    //tbot.set_pose(x_odd, y_odd, theta_odd);
    tbot.set_pose(x_true, y_true, theta_true);
    tbot.spin();
    if (tbot.get_goal_idx() > last_goal_idx){
      last_goal_idx = tbot.get_goal_idx();
    }
    twist.linear.x = tbot.get_v();
    twist.angular.z = tbot.get_w();
    twist_pub.publish(twist);
    update_goals_markers(node, tbot);
		loop_rate.sleep();
	}
	return 0;
}
