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


#ifndef EXPO_MOTIONPLANNER_HPP
#define EXPO_MOTIONPLANNER_HPP


#include <sfl/api/MotionPlanner.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>


namespace sfl {
  class DynamicWindow;
  class SpeedObjective;
  class HeadingObjective;
  class Multiscanner;
  class RobotModel;
  class BubbleBand;
  class Odometry;
  class Multiscanner;
}


namespace expo {
  
  
  class MotionPlannerStateMachine;
  class MotionController;
  
  
  /** \note lots of public fields for historical reasons... */
  class MotionPlanner:
    public sfl::MotionPlanner
  {
  public:
    enum state_id_t {
      take_aim,	           //!< rotate on spot until aligned with path
      aimed,	           //!< follow path with forward or backward speed
      adjust_goal_heading, //!< rotate on spot until aligned with goal theta
      at_goal,		   //!< stand still (react if pushed away though)
      manual_stop,	   //!< brake until standstill
      invalid	           //!< something went wrong (bug in state machine)
    };
    
    MotionPlanner(boost::shared_ptr<MotionController> motion_controller,
		  boost::shared_ptr<sfl::DynamicWindow> dynamic_window,
		  boost::shared_ptr<sfl::SpeedObjective> speed_objective,
		  boost::shared_ptr<sfl::HeadingObjective> heading_objective,
		  boost::shared_ptr<sfl::Multiscanner> multiscanner,
		  boost::shared_ptr<const sfl::RobotModel> robot_model,
		  /** optional: if null then go straight towards goal */
		  boost::shared_ptr<sfl::BubbleBand> bubble_band,
		  boost::shared_ptr<const sfl::Odometry> odometry);
    
    void Update(double timestep);
    void SetGoal(double timestep, const sfl::Goal & goal);
    const sfl::Goal & GetGoal() const;
    bool GoalReached() const;
    state_id_t GetStateId() const;
    const char * GetStateName() const;
    void GoForward();
    void GoBackward();
    
    /**
       If the angles are appropriate, sets the fields
       dtheta_starthoming and dtheta_startaiming.
       
       \return true iff 0 < aiming < homing
    */
    bool SetAimingThresholds(/** angle [rad] at which we switch to
				 'pure rotation until aligned with
				 path' mode */
			     double aiming,
			     /** angle [rad] at which we switch to
				 'follow path to goal' mode */
			     double homing);
    
    /**
       Triggers ManualStopState, which makes the robot brake with
       maximum allowed acceleration. The robot resumes motion as soon
       as a new goal is defined with SetGoal(), or if you call
       ManualResume().
    */
    void ManualStop();
    
    /**
       If we are in ManualStopState, this method makes the robot
       resume to the goal it was aiming for at the time of
       ManualStop().
    */
    void ManualResume();

    /** \note Hack for Cogniron, does ugly things like const_casts!
	\return <ul><li>  0: success                        </li>
                    <li> -1: odometry update error          </li>
		    <li> -2: multiscanner update error      </li>
		    <li> -3: motion controller update error </li></ul> */
    int UpdateAll(double timestep);
    
    boost::shared_ptr<MotionController> motion_controller;
    boost::shared_ptr<sfl::DynamicWindow> dynamic_window;
    boost::shared_ptr<sfl::SpeedObjective> speed_objective;
    boost::shared_ptr<sfl::HeadingObjective> heading_objective;
    boost::shared_ptr<const sfl::RobotModel> robot_model;
    boost::shared_ptr<sfl::BubbleBand> bubble_band; // can be null!
    boost::shared_ptr<const sfl::Odometry> odometry;
    boost::shared_ptr<sfl::Multiscanner> multiscanner;
    
    boost::scoped_ptr<sfl::Goal> goal;
    bool go_forward;
    
    double dtheta_starthoming; 	//!< default 10 * M_PI / 180
    double dtheta_startaiming;	//!< default 45 * M_PI / 180;
    
  private:
    MotionPlannerStateMachine * m_state_machine;
    ////unused?////    bool m_replan_request;
  };

}

#endif // EXPO_MOTIONPLANNER_HPP
