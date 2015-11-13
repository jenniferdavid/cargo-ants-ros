/* 
 * Copyright (C) 2005
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#ifndef THEATER_MOTION_MANAGER_HPP
#define THEATER_MOTION_MANAGER_HPP


#include <theater/MotionManager.hpp>


namespace expo {
  class MotionPlanner;
}

namespace sfl {
  class DynamicWindow;
  class Multiscanner;
  class MotionController;
  class Odometry;
  class Objective;
  class RobotModel;
}


class TheaterMotionManager:
  public MotionManager
{
public:
  TheaterMotionManager(expo::MotionPlanner & motion_planner,
		       sfl::DynamicWindow & dynamic_window,
		       sfl::Multiscanner & multiscanner,
		       sfl::MotionController & motion_controller,
		       sfl::Odometry & odometry,
		       sfl::RobotModel & robot_model);
  
  
  virtual bool PrepareAction(double timestamp, std::ostream & os);
  virtual void UseObstacleAvoidance(double x, double y, double theta,
				    double dr, double dtheta);
  virtual void UseBaseBehavior(BaseBehavior * bb);
  virtual void Standstill();
  
  virtual bool GoalReached();
  virtual double GetX();
  virtual double GetY();
  virtual double GetTheta();
  
  virtual void GlobalToActuators(double sd, double thetad,
				 double & qdl, double & qdr);
  virtual void ActuatorsToGlobal(double qdl, double qdr,
				 double & sd, double & thetad);

  
private:
  friend class TheaterRobot;
  
  
  typedef enum {
    NONE,
    OBSTAVOID,
    BASE
  } mode_t;
  
  
  void HackedAlphaHeading(sfl::Objective * ho, double val);
  void HackedAlphaBSpline(sfl::Objective * bso, double val);
  void HackedAlphaStuff(bool use_heading);
  
  
  sfl::RobotModel & _robot_model;
  expo::MotionPlanner & _motion_planner;
  sfl::DynamicWindow & _dynamic_window;
  sfl::Multiscanner & _multiscanner;
  sfl::MotionController & _motion_controller;
  sfl::Odometry & _odometry;

  sfl::Objective * _ho;
  double _ho_sticky_alpha;
  sfl::Objective * _bso;
  double _bso_sticky_alpha;

  mode_t _mode;
  BaseBehavior * _base_behavior;
};

#endif // THEATER_MOTION_MANAGER_HPP
