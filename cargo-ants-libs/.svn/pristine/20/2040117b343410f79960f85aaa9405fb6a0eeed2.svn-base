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


#include "ReplanHandler.hpp"
#include "BubbleList.hpp"
#include "BubbleBand.hpp"
#include "BubbleFactory.hpp"
#include <sfl/api/Pose.hpp>
#include <sfl/gplan/NF1.hpp>
#include <sfl/gplan/NF1Wave.hpp>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  const double ReplanHandler::DEFAULTNF1WIDTH(4.0);


  ReplanHandler::
  ReplanHandler(BubbleBand & bubble_band,
		BubbleFactory & bubble_factory)
    : m_bubble_band(bubble_band),
      m_bubble_factory(bubble_factory),
      m_nf1(new NF1()),
      m_buffer_blist(new BubbleList(bubble_band,
				    bubble_factory,
				    bubble_band.parameters)),
      m_nf1width(DEFAULTNF1WIDTH),
      m_nf1dimension(DEFAULTNF1DIMENSION)
  {
  }
  
  
  ReplanHandler::
  ~ReplanHandler()
  {
    delete m_buffer_blist;
  }
  
  
  bool ReplanHandler::
  GeneratePlan(shared_ptr<const Frame> pose, shared_ptr<const Scan> scan)
  {
    m_nf1->Configure(pose->X(), pose->Y(),
		     m_bubble_band.GlobalGoal().X(),
		     m_bubble_band.GlobalGoal().Y(),
		     m_nf1width,
		     m_nf1dimension);
    m_nf1->Initialize(scan,
		      m_bubble_band.robot_radius,
		      m_bubble_band.NF1GoalRadius());
    m_nf1->Calculate();
    const bool ok(m_nf1->ResetTrace());
    if(ok){
      m_nf1width = DEFAULTNF1WIDTH;
      return true;
    }
    m_nf1width *= 2;
    return false;
  }
  
  
  bool ReplanHandler::
  GenerateBand(shared_ptr<const Frame> pose, shared_ptr<const Scan> scan)
  {
    m_buffer_blist->RemoveAll();
    
    Bubble * bubble(m_bubble_factory.New(m_bubble_band.ReactionRadius(),
					 pose->X(), pose->Y()));
    if(bubble == 0)
      return false;
    
    bubble->_alpha_int = 0;
    bubble->_alpha_ext = 0;
    
    m_buffer_blist->Append(bubble);
    
    vec2d<double> point;
    while(m_nf1->GlobalTrace(point)){
      bubble = m_bubble_factory.New(m_bubble_band.ReactionRadius(),
				    point.v0, point.v1);
      
      if(bubble == 0){
	return false;
      }
      
      m_buffer_blist->Append(bubble);
    }
    
    bubble = m_bubble_factory.New(m_bubble_band.ReactionRadius(),
				  m_bubble_band.GlobalGoal().X(),
				  m_bubble_band.GlobalGoal().Y());
    if(bubble == 0){
      return false;
    }
    
    bubble->_alpha_int = 0;
    bubble->_alpha_ext = 0;
    bubble->SetMinIgnoreDistance(m_bubble_band.MinIgnoreDistance());
    
    m_buffer_blist->Append(bubble);
    
    if( ! m_buffer_blist->Update(scan)){
      return false;
    }
    
    return true;
  }
  
  
  BubbleList * ReplanHandler::
  SwapBubbleList(BubbleList * replace)
  {
    BubbleList * result(m_buffer_blist);
    replace->RemoveAll();
    m_buffer_blist = replace;
    return result;
  }
  
}
