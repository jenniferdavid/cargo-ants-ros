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


#include "BubbleBand.hpp"
#include "BubbleFactory.hpp"
#include "ReplanHandler.hpp"
#include <sfl/api/Scan.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Pose.hpp>
#include <iostream>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  BubbleBand::
  BubbleBand(const RobotModel & robot_model,
	     const Odometry & odometry,
	     const Multiscanner & multiscanner,
	     BubbleList::Parameters _parameters)
    : parameters(_parameters),
      robot_radius(robot_model.GetHull()->CalculateRadius()),
      robot_diameter(2 * robot_radius),
      ignore_radius(0.9 * robot_radius),
      deletion_diameter(1.8 * robot_diameter),
      addition_diameter(1.2 * robot_diameter),
      m_odometry(odometry),
      m_multiscanner(multiscanner),
      m_bubble_factory(new BubbleFactory(50)), // hax: hardcoded initlevel
      m_replan_handler(new ReplanHandler(*this, *m_bubble_factory)),
      m_frame(new Frame()),
      m_active_blist(new BubbleList(*this, *m_bubble_factory, _parameters)),
      m_reaction_radius(2.0 * robot_radius),
      m_replan_request(false),
      m_state(NOBAND),
      m_planstep(IDLE)
  {
  }
  
  
  BubbleBand::
  BubbleBand(const RobotModel & robot_model,
	     const Odometry & odometry,
	     const Multiscanner & multiscanner,
	     boost::shared_ptr<ReplanHandlerAPI> replan_handler,
	     BubbleList::Parameters _parameters)
    : parameters(_parameters),
      robot_radius(robot_model.GetHull()->CalculateRadius()),
      robot_diameter(2 * robot_radius),
      ignore_radius(0.9 * robot_radius),
      deletion_diameter(1.8 * robot_diameter),
      addition_diameter(1.2 * robot_diameter),
      m_odometry(odometry),
      m_multiscanner(multiscanner),
      m_bubble_factory(new BubbleFactory(50)), // hax: hardcoded initlevel
      m_replan_handler(replan_handler),
      m_frame(new Frame()),
      m_active_blist(new BubbleList(*this, *m_bubble_factory, _parameters)),
      m_reaction_radius(2.0 * robot_radius),
      m_replan_request(false),
      m_state(NOBAND),
      m_planstep(IDLE)
  {
  }
  
  
  BubbleBand::
  ~BubbleBand()
  {
    delete m_active_blist;
  }
  

  void BubbleBand::
  SetGoal(const Goal & global_goal)
  {
    m_min_ignore_distance = 0;
    m_nf1_goal_radius = global_goal.Dr();
    m_replan_request = true;
    m_global_goal = global_goal;
  }
  
  
  bool BubbleBand::
  AppendGoal(const Goal & global_goal, shared_ptr<const Scan> scan)
  {
    m_min_ignore_distance = 0;
    m_nf1_goal_radius = global_goal.Dr();
    m_global_goal = global_goal;
    if(m_active_blist->m_head == 0){ // no band to append to...
      m_replan_request = true;
      return false;
    }
    
    // place bubble at new goal, after verifying there's enough space
    Bubble *bubble = m_bubble_factory->New(m_reaction_radius,
					   m_global_goal.X(),
					   m_global_goal.Y());
    if(bubble == 0){
      m_active_blist->RemoveAll();
      m_replan_request = true;
      return false;
    }
    Bubble *tail(m_active_blist->m_tail);
    bubble->UpdateExternalParameters(scan, ignore_radius);
    bubble->_ignore_distance = tail->_ignore_distance;
    bubble->_dprevious = Bubble::Distance(*bubble, *tail);
    bubble->_dnext = -1;
    if(Bubble::CheckOverlap(*tail, *bubble, robot_radius)){
      tail->_alpha_int = Bubble::DEFAULTALPHAINT;
      tail->_alpha_ext = Bubble::DEFAULTALPHAEXT;
      tail->_dnext = bubble->_dprevious;
      tail->_fint.first
	= (bubble->_position.first - tail->_position.first) / tail->_dnext;
      tail->_fint.second
	= (bubble->_position.second - tail->_position.second) / tail->_dnext;
      m_active_blist->Append(bubble);
      return true;
    }
    
    // couldn't fix existing band
    m_bubble_factory->Delete(bubble);
    m_active_blist->RemoveAll();
    m_replan_request = true;
    return false;
  }
  
  
  bool BubbleBand::
  AppendTarget(const Goal & global_goal)
  {
    m_min_ignore_distance = global_goal.Dr() + robot_radius;
    m_nf1_goal_radius = m_min_ignore_distance;
    m_global_goal = global_goal;
    if(m_active_blist->m_head == 0){
      m_replan_request = true;
      return false;
    }
    
    // cheap method: move the last bubble onto the target, attempt to
    // keep a copy on its previous position, don't even check if
    // there's enough clearance (next update should take care of that...)
    Bubble *bubble = m_bubble_factory->Clone(m_active_blist->m_tail);
    if(bubble != 0){
      bubble->SetMinIgnoreDistance(0);
      bubble->_alpha_int = Bubble::DEFAULTALPHAINT;
      bubble->_alpha_ext = Bubble::DEFAULTALPHAEXT;
      m_active_blist->InsertAfter(m_active_blist->m_tail->_previous, bubble);
    }
    m_active_blist->m_tail->_position =
      make_pair(m_global_goal.X(), m_global_goal.Y());
    return true;
  }
  
  
  void BubbleBand::
  Update()
  {
    /* The Scan object should be filtered, ie contain only
       valid readings. This can be obtained from
       Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
       can still contain readings that are out of range (represented
       as readings at the maximum rho value). */
    shared_ptr<const Scan> scan(m_multiscanner.CollectScans());
    
    if(m_replan_request){
      m_active_blist->RemoveAll();
      m_state = NOBAND;
      m_replan_request = false;
      m_planstep = CREATE_PLAN;
      return;
    }
    
    m_frame = m_odometry.Get();
    if(m_active_blist->m_head != 0){
      m_active_blist->m_head->_position.first = m_frame->X();
      m_active_blist->m_head->_position.second = m_frame->Y();
    }
    
    if((CREATE_PLAN == m_planstep)
       && m_replan_handler->GeneratePlan(m_frame, scan))
      m_planstep = CREATE_BAND;
    
    if(CREATE_BAND == m_planstep){
      if( ! m_replan_handler->GenerateBand(m_frame, scan))
	m_planstep = CREATE_PLAN;
      else{
	m_active_blist = m_replan_handler->SwapBubbleList(m_active_blist);
	m_state = VALIDBAND;
	m_planstep = IDLE;
	return;
      }
    }
    
    if(m_active_blist->Empty())
      m_state = NOBAND;
    else{
      if(m_active_blist->Update(scan))
	m_state = VALIDBAND;
      else{
	m_state = UNSUREBAND;
	m_planstep = CREATE_PLAN;
      }
    }
  }
  
  
  void BubbleBand::
  GetSubGoal(double carrot_distance, double & goalx, double & goaly) const
  {
    if((m_active_blist->m_head == 0)
       || (m_active_blist->m_head == m_active_blist->m_tail)){
      goalx = m_global_goal.X();
      goaly = m_global_goal.Y();
      return;
    }
    
    carrot_distance *= carrot_distance; // avoid sqrt() calls
    for(Bubble *b(m_active_blist->m_head->_next); b != 0; b = b->_next){
      double dx(b->_position.first - m_frame->X());
      double dy(b->_position.second - m_frame->Y());
      if(dx * dx + dy * dy >= carrot_distance){
	goalx = b->_position.first;
	goaly = b->_position.second;
	return;
      }
    }
    
    goalx = m_global_goal.X();
    goaly = m_global_goal.Y();
  }
  
}
