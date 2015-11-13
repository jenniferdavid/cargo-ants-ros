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


#include "TheaterMotionManager.hpp"
#include <theater/BaseBehavior.hpp>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/Objective.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/MotionController.hpp>
#include <sfl/api/Odometry.hpp>
#include <iostream>

using namespace std;


TheaterMotionManager::
TheaterMotionManager(expo::MotionPlanner & motion_planner,
		     sfl::DynamicWindow & dynamic_window,
		     sfl::Multiscanner & multiscanner,
		     sfl::MotionController & motion_controller,
		     sfl::Odometry & odometry,
		     sfl::RobotModel & robot_model):
  _robot_model(robot_model),
  _motion_planner(motion_planner),
  _dynamic_window(dynamic_window),
  _multiscanner(multiscanner),
  _motion_controller(motion_controller),
  _odometry(odometry),
  _ho(0),
  _bso(0),
  _mode(NONE),
  _base_behavior(0)
{
}


bool TheaterMotionManager::
PrepareAction(double timestamp,
	      ostream & os)
{
  switch(_mode){

  case NONE:
    _motion_controller.ProposeActuators(0, 0);
    break;
    
  case OBSTAVOID:
    _motion_planner.Update();
    if(_motion_planner.GoalReached())
      _mode = NONE;
    break;
    
  case BASE:
    if(_base_behavior == 0){
      os << "ERROR in TheaterMotionManager::PrepareAction():\n"
	 << "  _base_behavior == 0 in BASE mode.\n";
      return false;
    }
    {
      double qdl, qdr;
      _base_behavior->GetActuators(timestamp, qdl, qdr);
      
      os << "DEBUG TheaterMotionManager::PrepareAction():"
	 << " wheelspeed : (" << qdl << ", " << qdr << ")\n";
      
      _motion_controller.ProposeActuators(qdl, qdr);
    }
    break;

    //   case BSPLINE:
    //     if(_bspline_behavior == 0){
    //       os << "ERROR in TheaterMotionManager::PrepareAction():\n"
    // 	 << "  _bspline_behavior == 0 in BSPLINE mode.\n";
    //       return false;
    //     }
    //     {
    //       os << "DEBUG TheaterMotionManager::PrepareAction(): Using carrot.\n";
      
    //       double x, y, sd;
    //       _bspline_behavior->GetCarrot(timestamp, x, y, sd);
    
    //       os << "DEBUG TheaterMotionManager::PrepareAction(): carrot = " << x
    // 	 << "\t" << y << "\t" << sd <<  ".\n";
      
    //       _odometry.Get().From(x, y);
      
    //       os << "DEBUG TheaterMotionManager::PrepareAction(): local  = " << x
    // 	 << "\t" << y <<  ".\n";
      
    //       _dynamic_window.OverrideSd(sd);
    //       _dynamic_window.Update(x, y, _multiscanner.CollectGlobalScans(_odometry.Get()));
    //       double qdl, qdr;
    //       if( ! _dynamic_window.OptimalActuators(qdl, qdr)){
    // 	os << "WARNING in TheaterMotionManager::PrepareAction():\n"
    // 	   << "  No optimal actuator command found!\n";
    // 	qdl = 0;
    // 	qdr = 0;
    //       }
    
    //       os << "DEBUG TheaterMotionManager::PrepareAction():"
    // 	 << " wheelspeed : (" << qdl << ", " << qdr << ")\n";
      
    //       _motion_controller.ProposeActuators(qdl, qdr);
    //     }
    //     break;
    
  default:
    os << "ERROR in TheaterMotionManager::PrepareAction():\n"
       << "  unhandled mode " << _mode << "\n";
    return false;
  }

  return true;
}


bool TheaterMotionManager::
GoalReached()
{
  if(_mode == OBSTAVOID)
    return _motion_planner.GoalReached();
  return false;
}


void TheaterMotionManager::
HackedAlphaHeading(sfl::Objective * ho,
		   double val)
{
  _ho = ho;
  _ho_sticky_alpha = val;
}


void TheaterMotionManager::
HackedAlphaBSpline(sfl::Objective * bso,
		   double val)
{
  _bso = bso;
  _bso_sticky_alpha = val;
}


void TheaterMotionManager::
HackedAlphaStuff(bool use_heading)
{
  if(_ho == 0){
    cerr << "ERROR in TheaterMotionManager::HackedAlphaStuff(): Sticky ho?\n";
    exit(EXIT_FAILURE);
  }
  if(_bso == 0){
    cerr << "ERROR in TheaterMotionManager::HackedAlphaStuff(): Sticky bso?\n";
    exit(EXIT_FAILURE);
  }
  
  if(use_heading){
    _ho->SetAlpha(_ho_sticky_alpha);
    _bso->SetAlpha(0);
  }
  else{
    _ho->SetAlpha(0);
    _bso->SetAlpha(_bso_sticky_alpha);
  }
}


void TheaterMotionManager::
Standstill()
{
  _mode = NONE;
}


void TheaterMotionManager::
UseObstacleAvoidance(double x,
		     double y,
		     double theta,
		     double dr,
		     double dtheta)
{
  _mode = OBSTAVOID;
  _motion_planner.SetGoal(sfl::Goal(x, y, theta, dr, dtheta));
  HackedAlphaStuff(true);
}


void TheaterMotionManager::
UseBaseBehavior(BaseBehavior * bb)
{
  _mode = BASE;
  _base_behavior = bb;
  HackedAlphaStuff(false);
}


double TheaterMotionManager::
GetX()
{
  return _odometry.Get().X();
}


double TheaterMotionManager::
GetY()
{
  return _odometry.Get().Y();
}


double TheaterMotionManager::
GetTheta()
{
  return _odometry.Get().Theta();
}


void TheaterMotionManager::
GlobalToActuators(double sd,
		  double thetad,
		  double & qdl,
		  double & qdr)
{
  _robot_model.Global2Actuator(sd, thetad, qdl, qdr);
}


void TheaterMotionManager::
ActuatorsToGlobal(double qdl,
		  double qdr,
		  double & sd,
		  double & thetad)
{
  _robot_model.Actuator2Global(qdl, qdr, sd, thetad);
}
