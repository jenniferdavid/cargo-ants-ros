/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#include "MotionPlanner.hpp"
#include "MotionController.hpp"
#include "MotionPlannerState.hpp"

// manual
#define SFL_DEBUG 1
#include "../util/pdebug.hpp"

#include "../api/Multiscanner.hpp"
#include "../api/Goal.hpp"
#include "../api/Pose.hpp"
#include "../api/Odometry.hpp"
#include "../bband/BubbleBand.hpp"
#include "../dwa/DynamicWindow.hpp"
#include "../dwa/SpeedObjective.hpp"
#include "../dwa/HeadingObjective.hpp"
#include <cmath>

#ifdef WIN32
# include "../util/numeric.hpp"
#endif // WIN32


using namespace boost;


namespace expo {
  
  
  MotionPlanner::
  MotionPlanner(shared_ptr<MotionController> _motion_controller,
		shared_ptr<sfl::DynamicWindow> _dynamic_window,
		shared_ptr<sfl::SpeedObjective> _speed_objective,
		shared_ptr<sfl::HeadingObjective> _heading_objective,
		shared_ptr<sfl::Multiscanner> _multiscanner,
		shared_ptr<const sfl::RobotModel> _robot_model,
		shared_ptr<sfl::BubbleBand> _bubble_band,
		shared_ptr<const sfl::Odometry> _odometry)
    : motion_controller(_motion_controller),
      dynamic_window(_dynamic_window),
      speed_objective(_speed_objective),
      heading_objective(_heading_objective),
      robot_model(_robot_model),
      bubble_band(_bubble_band),
      odometry(_odometry),
      multiscanner(_multiscanner),
      goal(new sfl::Goal()),
      go_forward(true),
      dtheta_starthoming(10 * M_PI / 180),
      dtheta_startaiming(45 * M_PI / 180),
      m_state_machine(new MotionPlannerStateMachine(this))
      ////unused?//// m_replan_request(false)
  {
  }
  
  
  void MotionPlanner::
  Update(double timestep)
  {
    PDEBUG("\n==================================================\n");
    shared_ptr<const sfl::Pose> pose(odometry->Get());
    m_state_machine->Next(timestep);
    m_state_machine->current_state->Act(timestep,
					multiscanner->CollectScans());
  }
  
  
  void MotionPlanner::
  SetGoal(double timestep, const sfl::Goal & _goal)
  {
    goal->Set(_goal);
    if(bubble_band)
      bubble_band->SetGoal(_goal);
    m_state_machine->GoalChanged(timestep);
  }
  
  
  const sfl::Goal & MotionPlanner::
  GetGoal() const
  {
    return * goal;
  }
  
  
  bool MotionPlanner::
  GoalReached() const
  {
    shared_ptr<const sfl::Pose> pose(odometry->Get());
    if( ! goal->Reached( * pose, go_forward))
      return false;
    if(goal->IsVia())
      return true;
    return ! motion_controller->Moving();
  }
  
  
  int MotionPlanner::
  UpdateAll(double timestep)
  {
    PDEBUG("\n==================================================\n");
    sfl::Odometry & odo(const_cast<sfl::Odometry &>(*odometry));
    int status(odo.Update());
    if(0 > status){
      PDEBUG("ERROR odo.Update(): %d\n", status);
      return -1;
    }
    if( ! multiscanner->UpdateAll()){
      PDEBUG("ERROR multiscanner->UpdateAll() failed\n");
      return -2;
    }
    Update(timestep);
    status = motion_controller->Update(timestep);
    if(0 > status){
      PDEBUG("ERROR motion_controller->Update(): %d\n", status);
      return -3;
    }
    return 0;
  }
  
  
  MotionPlanner::state_id_t MotionPlanner::
  GetStateId() const
  {
    return m_state_machine->GetStateId();
  }
  
  
  const char * MotionPlanner::
  GetStateName() const
  {
    return m_state_machine->GetStateName();
  }
  
  
  void MotionPlanner::
  GoForward()
  {
    go_forward = true;
    heading_objective->angle_offset = 0;
    speed_objective->GoForward();
  }
  
  
  void MotionPlanner::
  GoBackward()
  {
    go_forward = false;
    heading_objective->angle_offset = M_PI;
    speed_objective->GoBackward();
  }
  
  
  bool MotionPlanner::
  SetAimingThresholds(double aiming, double homing)
  {
    if((0 < aiming) && (aiming > homing)){
      dtheta_starthoming = homing;
      dtheta_startaiming = aiming;
      return true;
    }
    return false;
  }
  
  
  void MotionPlanner::
  ManualStop()
  {
    m_state_machine->ManualStop();
  }
  
  
  void MotionPlanner::
  ManualResume()
  {
    m_state_machine->ManualResume();
  }
  
}
