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


#include "MotionPlannerState.hpp"
#include "MotionPlanner.hpp"
#include "MotionController.hpp"
#include "../util/numeric.hpp"

// manual override
#define SFL_DEBUG
#include "../util/pdebug.hpp"

#include "../api/Pose.hpp"
#include "../api/Odometry.hpp"
#include "../bband/BubbleBand.hpp"
#include "../dwa/DynamicWindow.hpp"
#include "../dwa/SpeedObjective.hpp"
#include "../dwa/DistanceObjective.hpp"
#include <cmath>


using namespace boost;
using namespace std;


namespace expo {
  
  
  /**
     Rotate on spot until aligned within
     expo::MotionPlanner::dtheta_starthoming of path direction.
  */
  class TakeAimState
    : public MotionPlannerState
  {
  private:
    TakeAimState(const TakeAimState &); //!< non-copyable
    
  public:
    explicit TakeAimState(MotionPlanner * mp,
			  MotionPlannerStateMachine * sm);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t GetPathDirection();

  private:
    double dheading;
  
    bool StartHoming(double dtheta) const;
  };


  /**
     Follow path with forward or backward speed, switched to
     TakeAimState if the angle between the robot's translational
     movement and the path becomes greater than
     expo::MotionPlanner::dtheta_startaiming.
  */
  class AimedState
    : public MotionPlannerState
  {
  private:
    AimedState(const AimedState &); //!< non-copyable
    
  public:
    explicit AimedState(MotionPlanner * mp,
			MotionPlannerStateMachine * sm);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t GetPathDirection();

  private:
    double dheading;

    bool StartAiming(double dtheta) const;
    bool Replan() const;
  };


  /**
     Rotate on spot until aligned with goal theta, within the goal's
     dtheta parameter.
  */
  class AdjustGoalHeadingState
    : public MotionPlannerState
  {
  private:
    AdjustGoalHeadingState(const AdjustGoalHeadingState &); //!< non-copyable
    
  public:
    explicit AdjustGoalHeadingState(MotionPlanner * mp,
				    MotionPlannerStateMachine * sm);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);

  protected:
    direction_t GetPathDirection();
  };

  
  /**
     Stand still, but react if pushed away: if necessary, switch back
     to another state such as AdjustGoalHeadingState.
  */
  class AtGoalState
    : public MotionPlannerState
  {
  private:
    AtGoalState(const AtGoalState &); //!< non-copyable
    
  public:
    explicit AtGoalState(MotionPlanner * mp,
			 MotionPlannerStateMachine * sm);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);
  };
  
  
  /**
     Brake until standstill.
  */
  class ManualStopState
    : public MotionPlannerState
  {
  private:
    ManualStopState(const ManualStopState &); //!< non-copyable
    
  public:
    explicit ManualStopState(MotionPlanner * mp,
			     MotionPlannerStateMachine * sm);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);
  };
  
  
  MotionPlannerState::
  MotionPlannerState(const string & name,
		     MotionPlanner * mp,
		     MotionPlannerStateMachine * sm)
    : m_mp(mp),
      m_sm(sm),
      m_name(name)
  {
  }
  
  
  MotionPlannerState::
  ~MotionPlannerState()
  {
  }
  
  
  const string & MotionPlannerState::
  Name() const
  {
    return m_name;
  }
  
  
  MotionPlannerState * MotionPlannerState::
  NextState(double timestep)
  {
    shared_ptr<const sfl::Pose> pose(m_mp->odometry->Get());
    if(m_mp->goal->DistanceReached( * pose)){
      double dheading;
      if(m_mp->goal->HeadingReached( * pose, m_mp->go_forward, dheading)
	 && (m_mp->motion_controller->Stoppable(timestep)
	     || m_mp->goal->IsVia()))
	return m_sm->at_goal_state.get();
      return m_sm->adjust_goal_heading_state.get();      
    }
    return this;
  }
  
  
  MotionPlannerStateMachine::
  MotionPlannerStateMachine(MotionPlanner * mp)
    : manual_stop_state(new ManualStopState(mp, this)),
      take_aim_state(new TakeAimState(mp, this)),
      aimed_state(new AimedState(mp, this)),
      adjust_goal_heading_state(new AdjustGoalHeadingState(mp, this)),
      at_goal_state(new AtGoalState(mp, this)),
      current_state(at_goal_state.get()),
      m_mp(mp),
      m_manual_state_latch(current_state)
  {
  }
  
  
  void MotionPlannerStateMachine::
  FollowTarget()
  {
    current_state = aimed_state.get();
  }
  
  
  void MotionPlannerStateMachine::
  Next(double timestep)
  {
    current_state = current_state->NextState(timestep);
  }
  
  
  void MotionPlannerStateMachine::
  GoalChanged(double timestep)
  {
    shared_ptr<const sfl::Pose> pose(m_mp->odometry->Get());
    if(m_mp->goal->DistanceReached( * pose)){
      double dheading;
      if(m_mp->goal->HeadingReached( * pose, m_mp->go_forward, dheading)
	 && (m_mp->motion_controller->Stoppable(timestep)
	     || m_mp->goal->IsVia())){
	current_state = at_goal_state.get();
	return;
      }
      current_state = adjust_goal_heading_state.get();
      return;
    }
    const double dx(m_mp->goal->X() - pose->X());
    const double dy(m_mp->goal->Y() - pose->Y());
    double dtheta(sfl::mod2pi(atan2(dy, dx) - pose->Theta()));
    if( ! m_mp->go_forward)
      dtheta = M_PI - dtheta;
    if(dtheta < m_mp->dtheta_startaiming){
      current_state = aimed_state.get();
      return;
    }
    current_state = take_aim_state.get();
  }
  
  
  void MotionPlannerStateMachine::
  ManualStop()
  {
    if(current_state == manual_stop_state.get())
      return;
    m_manual_state_latch = current_state;
    current_state = manual_stop_state.get();
  }
  
  
  void MotionPlannerStateMachine::
  ManualResume()
  {
    if(current_state != manual_stop_state.get())
      return;
    current_state = m_manual_state_latch;
  }
  
  
  MotionPlanner::state_id_t MotionPlannerStateMachine::
  GetStateId() const
  {
    if(current_state == take_aim_state.get())
      return MotionPlanner::take_aim;
    if(current_state == at_goal_state.get())
      return MotionPlanner::at_goal;
    if(current_state == aimed_state.get())
      return MotionPlanner::aimed;
    if(current_state == adjust_goal_heading_state.get())
      return MotionPlanner::adjust_goal_heading;
    if(current_state == at_goal_state.get())
      return MotionPlanner::at_goal;
    if(current_state == manual_stop_state.get())
      return MotionPlanner::manual_stop;
    return MotionPlanner::invalid;
  }
  
  
  const char * MotionPlannerStateMachine::
  GetStateName() const
  {
    if(current_state == manual_stop_state.get())
      return "MANUAL_STOP";
    if(current_state == take_aim_state.get())
      return "TAKE_AIM";
    if(current_state == aimed_state.get())
      return "AIMED";
    if(current_state == adjust_goal_heading_state.get())
      return "ADJUST_GOAL_HEADING";
    if(current_state == at_goal_state.get())
      return "AT_GOAL";
    return "<invalid>";
  }
  
  
  void MotionPlannerState::
  TurnToward(double timestep, direction_t direction,
	     shared_ptr<const sfl::Scan> global_scan) const
  {
    m_mp->speed_objective->GoSlow();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  void MotionPlannerState::
  GoAlong(double timestep, direction_t direction,
	  shared_ptr<const sfl::Scan> global_scan) const
  {
    m_mp->speed_objective->GoFast();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  MotionPlannerState::direction_t MotionPlannerState::
  AskBubbleBand() const
  {
    double goalx(m_mp->goal->X());
    double goaly(m_mp->goal->Y());
    if(m_mp->bubble_band){
      m_mp->bubble_band->Update();
      if(m_mp->bubble_band->GetState() != sfl::BubbleBand::NOBAND){
	m_mp->bubble_band->GetSubGoal(m_mp->bubble_band->robot_radius,
				      goalx, goaly);
	PDEBUG("expo MPS %s: goal %05.2f %05.2f   bband %05.2f %05.2f\n",
	       m_name.c_str(), m_mp->goal->X(), m_mp->goal->Y(),
	       goalx, goaly);
      }
      else
	PDEBUG("expo MPS %s: goal %05.2f %05.2f  NO BBAND\n",
	       m_name.c_str(), goalx, goaly);
    }
    else
      PDEBUG("expo MPS %s: goal %05.2f %05.2f  bband disabled\n",
	     m_name.c_str(), goalx, goaly);
    m_mp->odometry->Get()->From(goalx, goaly);
    return make_pair(goalx, goaly);
  }


  void MotionPlannerState::
  AskDynamicWindow(double timestep,
		   direction_t direction,
		   shared_ptr<const sfl::Scan> global_scan) const
  {
    double qdl, qdr;
    m_mp->motion_controller->GetCurrentAct(qdl, qdr);
    sfl::DynamicWindow & dwa(*m_mp->dynamic_window);
    dwa.Update(qdl, qdr, timestep, direction.first, direction.second, global_scan);
    
    if( ! dwa.OptimalActuators(qdl, qdr)){
      qdl = 0;
      qdr = 0;
      PDEBUG("expo MPS %s: DWA failed\n", m_name.c_str());
    }
    else
      PDEBUG("expo MPS %s: DWA %05.2f %05.2f\n", m_name.c_str(), qdl, qdr);
    
    m_mp->motion_controller->ProposeActuators(qdl, qdr);
  }


  TakeAimState::
  TakeAimState(MotionPlanner * mp,
	       MotionPlannerStateMachine * sm):
    MotionPlannerState("take aim", mp, sm)
  {
  }


  void TakeAimState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(), global_scan);
  }


  MotionPlannerState * TakeAimState::
  NextState(double timestep)
  {
    MotionPlannerState * override(MotionPlannerState::NextState(timestep));
    if(override != this)
      return override;

    if(StartHoming(dheading))
      return m_sm->aimed_state.get();

    return this;
  }


  MotionPlannerState::direction_t TakeAimState::
  GetPathDirection()
  {
    direction_t dir(AskBubbleBand());
    dheading = atan2(dir.second, dir.first);
    if( ! m_mp->go_forward) {
      if(dheading > 0)
	dheading =   M_PI - dheading;
      else
	dheading = - M_PI - dheading;
    }
    return dir;
  }


  bool TakeAimState::
  StartHoming(double dtheta) const
  {
    return (dtheta > 0 ? dtheta : - dtheta) <= m_mp->dtheta_starthoming;
  }


  AimedState::
  AimedState(MotionPlanner * mp,
	     MotionPlannerStateMachine * sm):
    MotionPlannerState("aimed", mp, sm)
  {
  }


  void AimedState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    GoAlong(timestep, GetPathDirection(), global_scan);
  }


  MotionPlannerState * AimedState::
  NextState(double timestep)
  {
    MotionPlannerState  *override(MotionPlannerState::NextState(timestep));
    if(override != this)
      return override;
  
    if(StartAiming(dheading))
      return m_sm->take_aim_state.get();
  
    return this;
  }


  MotionPlannerState::direction_t AimedState::
  GetPathDirection()
  {
    direction_t dir(AskBubbleBand());
    dheading = atan2(dir.second, dir.first);
    if( ! m_mp->go_forward) {
      if(dheading > 0)
	dheading =   M_PI - dheading;
      else
	dheading = - M_PI - dheading;
    }
    return dir;
  }


  bool AimedState::
  StartAiming(double dtheta) const
  {
    return (dtheta > 0 ? dtheta : - dtheta) >= m_mp->dtheta_startaiming;
  }


  AdjustGoalHeadingState::
  AdjustGoalHeadingState(MotionPlanner * mp,
			 MotionPlannerStateMachine * sm):
    MotionPlannerState("adjust goal heading", mp, sm)
  {
  }


  void AdjustGoalHeadingState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(), global_scan);
  }


  AdjustGoalHeadingState::direction_t AdjustGoalHeadingState::
  GetPathDirection()
  {
    shared_ptr<const sfl::Pose> pose(m_mp->odometry->Get());    
    double dtheta;
    m_mp->goal->HeadingReached( * pose, m_mp->go_forward, dtheta);
    return make_pair(cos(dtheta), sin(dtheta));
  }


  AtGoalState::
  AtGoalState(MotionPlanner * mp,
	      MotionPlannerStateMachine * sm):
    MotionPlannerState("at goal", mp, sm)
  {
  }


  void AtGoalState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    static const direction_t dir(1, 0);
    TurnToward(timestep, dir, global_scan);
  }


  MotionPlannerState * AtGoalState::
  NextState(double timestep)
  {
    MotionPlannerState  * override(MotionPlannerState::NextState(timestep));
    if(override != this)
      return override;
  
    return this;
  }
  
  
  ManualStopState::
  ManualStopState(MotionPlanner * mp,
		  MotionPlannerStateMachine * sm):
    MotionPlannerState("manual stop", mp, sm)
  {
  }
  
  
  void ManualStopState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    static const direction_t dir(1, 0);
    TurnToward(timestep, dir, global_scan);
  }
  
  
  MotionPlannerState * ManualStopState::
  NextState(double timestep)
  {
    return this;
  }
  
}
