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


#ifndef EXPO_MOTIONPLANNERSTATE_HPP
#define EXPO_MOTIONPLANNERSTATE_HPP


#include <sfl/expo/MotionPlanner.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <string>


namespace sfl {
  class Scan;
}


namespace expo {
  
  
  class MotionPlannerStateMachine;
  
  
  class MotionPlannerState
  {
  private:
    MotionPlannerState(const MotionPlannerState &); //!< non-copyable
    
  protected:
    MotionPlannerState(const std::string & name,
		       MotionPlanner * mp,
		       MotionPlannerStateMachine * sm);
  
  public:
    virtual ~MotionPlannerState();
  
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    virtual
    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan) = 0;
    virtual MotionPlannerState * NextState(double timestep);
    
    const std::string & Name() const;
    
  protected:
    typedef std::pair<double, double> direction_t;
    
    MotionPlanner * m_mp;
    MotionPlannerStateMachine * m_sm;
    const std::string m_name;

    void TurnToward(double timestep, direction_t local_direction,
		    boost::shared_ptr<const sfl::Scan> scan) const;
    
    void GoAlong(double timestep, direction_t local_direction,
		 boost::shared_ptr<const sfl::Scan> scan) const;
    
    direction_t AskBubbleBand() const;
    
    void AskDynamicWindow(double timestep, direction_t local_direction,
			  boost::shared_ptr<const sfl::Scan> scan) const;
  };
  
  
  class MotionPlannerStateMachine
  {
  public:
    explicit MotionPlannerStateMachine(MotionPlanner * mp);
    
    void Next(double timestep);
    void GoalChanged(double timestep);
    void FollowTarget();
    void ManualStop();
    void ManualResume();
    
    MotionPlanner::state_id_t GetStateId() const;
    const char * GetStateName() const;
    
    boost::scoped_ptr<MotionPlannerState> manual_stop_state;
    boost::scoped_ptr<MotionPlannerState> take_aim_state;
    boost::scoped_ptr<MotionPlannerState> aimed_state;
    boost::scoped_ptr<MotionPlannerState> adjust_goal_heading_state;
    boost::scoped_ptr<MotionPlannerState> at_goal_state;
    MotionPlannerState * current_state;
    
  private:
    MotionPlanner * m_mp;
    MotionPlannerState * m_manual_state_latch;
  };
  
}

#endif // EXPO_MOTIONPLANNERSTATE_HPP
