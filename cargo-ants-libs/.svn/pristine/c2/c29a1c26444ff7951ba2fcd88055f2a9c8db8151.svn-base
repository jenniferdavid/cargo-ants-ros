#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "diff_drive_robot.hpp"

#define PI 3.14159265359
#define K_RHO 0.5
#define K_DELTA -0.5
#define K_GAMMA 0.62
#define R_MERGE_START	1.0
#define R_MERGE_END	0.5


namespace diff_drive_robot {
DiffDriveRobot::DiffDriveRobot(double v_max, double w_max, double a_max, double alpha_max, double dt) :
	v_max_(v_max),
	w_max_(w_max),
	a_max_(a_max),
	alpha_max_(alpha_max),
  dt_(dt),
	goal_idx_(0),
	stop_mode_ (1)
  {};
void DiffDriveRobot::set_goals(std::vector<Goal> const &goals){
	//check if it's a new trajectory.
	// A better check is needed than the one below
	if (goals_.size()>0){
		if (hypot(goals.back().x - goals_.back().x,goals.back().y - goals_.back().y)> 0.5){
			goal_idx_ = 0; //reset goal counter
			goals_ = goals;
			stop_mode_ = 1;
			if (goals_.size()>1) { //chomp currently doesn't calculate heading for
														//trajectory points. It will be calculated here based
														//by forward propgating current headin	g.
				std::cout<<"end goal x = "<< goals_.back().x <<" , y = "<<goals_.back().y<<std::endl;
				goals_[0].theta = theta_;
				for (size_t ii=1; ii<goals_.size(); ++ii){
					goals_[ii].theta = atan2(goals_[ii].y-goals[ii-1].y,
																		goals_[ii].x-goals[ii-1].x);
				}
			}
		}
	}
	else { //goal.size() == 0
		goals_= goals;
		stop_mode_ = 1;
	}
}
void DiffDriveRobot::spin(){
	if (goals_.size()==0) return;
	if (stop_mode_){
		stop();
		return;
	}

  diff_drive_robot::Goal smoothed_goal, current_goal, next_goal;
  current_goal = goals_[goal_idx_];
	if ((distance_to_goal(current_goal)<R_MERGE_END) && (goal_idx_ < goals_.size() - 1)){
		goal_idx_++;
		current_goal = goals_[goal_idx_];
		std::cout<<"Moving to goal"<<goal_idx_<<std::endl;
	}
	std::cout<<"Distance to goal = "<<distance_to_goal(current_goal)<<std::endl;
  if (goal_idx_<goals_.size()-1)
    next_goal = goals_[goal_idx_+1];
  else
    next_goal = goals_[goal_idx_];
	//smooth transition between consecutive goals.

	if ((distance_to_goal(current_goal)<= R_MERGE_START) &&
					(distance_to_goal(current_goal)>= R_MERGE_END)){
						double d = distance_to_goal(current_goal);
						smoothed_goal.x = ((d-R_MERGE_END)/R_MERGE_END)*current_goal.x +
																((R_MERGE_START-d)/R_MERGE_END)*next_goal.x ;
						smoothed_goal.y = ((d-R_MERGE_END)/R_MERGE_END)*current_goal.y +
																((R_MERGE_START-d)/R_MERGE_END)*next_goal.y ;
						smoothed_goal.theta = ((d-R_MERGE_END)/R_MERGE_END)*current_goal.theta +
																((R_MERGE_START-d)/R_MERGE_END)*next_goal.theta;
					}
	else 	smoothed_goal = current_goal;	//for all other cases

  double dx = smoothed_goal.x-x_;
  double dy = smoothed_goal.y-y_;
  double eps = atan2(dy, dx);
  double rho = hypot(dx, dy);
  double gamma = wrap_angle(eps - theta_);
  double delta = wrap_angle(smoothed_goal.theta - gamma - theta_);
	std::cout<<"rho = "<<rho<<", gamma = "<<gamma*180.0/PI<<", delta = "<<delta*180.0/PI<<std::endl;
  double v_desired = K_RHO * rho;
  double w_desired = K_GAMMA * gamma + K_DELTA * delta;
	std::cout<<"v_desired = "<<v_desired<< ", w_desired = "<<w_desired*180.0/PI;
  // check velocity limits
  if (fabs(v_desired)>v_max_) v_desired = copysign(v_max_, v_desired);
  if (fabs(w_desired)>w_max_) w_desired = copysign(w_max_, w_desired);
  // check accelration limits
  if (fabs(v_desired-v_)/dt_ > a_max_)
    v_desired = v_ + copysign(a_max_*dt_, v_desired-v_);
  if (fabs(w_desired-w_)/dt_ > alpha_max_)
    w_desired = w_ + copysign(alpha_max_*dt_, w_desired-w_);
  w_ = w_desired;
	v_ = v_desired *(1.0 -0.25*fabs(w_)/w_max_);
  std::cout<<", v = "<<v_<<", w = "<<w_*180.0/PI<<std::endl;
}
void DiffDriveRobot::set_pose(double x, double y, double theta){
  x_ = x;
  y_ = y;
  theta_ = wrap_angle(theta);
}
double DiffDriveRobot::distance_to_goal(Goal const &goal) const{
	return hypot(goal.x - x_, goal.y - y_);
}

Goal DiffDriveRobot::get_goal(size_t idx) const{
		if (idx>= goals_.size()){
			Goal dummy;
			return dummy;
		}
		return goals_[idx];
}

void DiffDriveRobot::stop() {
	double v_desired = 0;
  double w_desired = 0;
  // check velocity limits
  if (fabs(v_desired)>v_max_) v_desired = copysign(v_max_, v_desired);
  if (fabs(w_desired)>w_max_) w_desired = copysign(w_max_, w_desired);
  // check accelration limits
  if (fabs(v_desired-v_)/dt_ > a_max_)
    v_desired = v_ + copysign(a_max_*dt_, v_desired-v_);
  if (fabs(w_desired-w_)/dt_ > alpha_max_)
    w_desired = w_ + copysign(alpha_max_*dt_, w_desired-w_);
  v_ = v_desired;
  w_ = w_desired;
	if (v_==0 && w_ == 0) stop_mode_ = 0;
}

double wrap_angle(double angle){
    // It's assumed that angle is between -2*pi and 2*pi, the returned
    // value will be between -pi and pi
    if (angle>PI) return -2*PI + angle;
    if (angle<-PI) return 2*PI + angle;
    return angle;
}
}
