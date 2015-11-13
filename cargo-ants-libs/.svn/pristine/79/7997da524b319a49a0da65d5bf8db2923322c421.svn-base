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


#include "BubbleList.hpp"
#include "BubbleBand.hpp"
#include "BubbleFactory.hpp"
#include <sfl/util/numeric.hpp>


namespace sfl {
  
  
  BubbleList::
  BubbleList(BubbleBand & bubble_band,
	     BubbleFactory & bubble_factory,
	     const Parameters & params):
    m_shortpath(params.shortpath),
    m_longpath(params.longpath),
    m_deltapath(params.longpath - params.shortpath),
    m_maxignoredistance(params.maxignoredistance),
    m_bubble_band(bubble_band),
    m_bubble_factory(bubble_factory),
    m_head(0),
    m_tail(0)
  {
  }
  
  
  BubbleList::
  BubbleList(BubbleList & original):
    m_shortpath(original.m_shortpath),
    m_longpath(original.m_longpath),
    m_deltapath(original.m_deltapath),
    m_maxignoredistance(original.m_maxignoredistance),
    m_bubble_band(original.m_bubble_band),
    m_bubble_factory(original.m_bubble_factory),
    m_head(0),
    m_tail(0)
  {
    for(Bubble * bubble(original.m_head);
	bubble->_next != 0;
	bubble = bubble->_next)
      {
	Bubble * nb(m_bubble_factory.Clone(bubble));
	while( ! nb){
	  m_bubble_factory.Produce(1);
	  nb = m_bubble_factory.Clone(bubble);
	}
	Append(nb);
      }
  }
  
  
  BubbleList::
  ~BubbleList()
  {
    RemoveAll();
  }
  
  
  bool BubbleList::
  Update(boost::shared_ptr<const Scan> scan)
  {
    if(m_head == 0)
      return false;		// do nothing
    
    UpdateBubbles(scan);	// this is not a good name!
    
    Bubble * bubble(m_head);
    while(bubble->_next != 0){
      while(CheckRemove(bubble))
	/* nop */;
      if(CheckAdd(bubble, scan)){
	if( ! (Bubble::CheckOverlap(*bubble,
				    *bubble->_next,
				    m_bubble_band.robot_diameter) &&
	       Bubble::CheckOverlap(*bubble->_next,
				    *bubble->_next->_next,
				    m_bubble_band.robot_diameter)))
	  {
	    return false;
	  }
	bubble = bubble->_next;
      }
      bubble = bubble->_next;
    }
    
    UpdatePathLength();
    UpdateBubbleBehaviour();
    
    return true;
  }
  
  
  void BubbleList::
  FirstBubble(Bubble * bubble)
  {
    bubble->_previous = 0;
    bubble->_next = 0;
    m_head = bubble;
    m_tail = bubble;
  }
  
  
  void BubbleList::
  Append(Bubble * bubble)
  {
    if(m_head == 0)
      FirstBubble(bubble);
    else{
      bubble->_previous = m_tail;
      bubble->_next = 0;
      m_tail->_next = bubble;
      m_tail = bubble;
    }
  }
  
  
  void BubbleList::
  InsertAfter(Bubble *current,
	      Bubble *fresh)
  {
    if(m_tail == current)
      Append(fresh);
    else{
      fresh->_previous = current;
      fresh->_next = current->_next;
      current->_next->_previous = fresh;
      current->_next = fresh;
    }
  }
  
  
  void BubbleList::
  Remove(Bubble *bubble)
  {
    if(bubble->_previous == 0){
      if(bubble->_next == 0){	// only element
	m_head = 0;
	m_tail = 0;
      }
      else{			// first element
	m_head = bubble->_next;
	m_head->_previous = 0;
      }
    }
    else{			// last element
      if(bubble->_next == 0){
	m_tail = bubble->_previous;
	m_tail->_next = 0;
      }
      else{			// normal element
	bubble->_previous->_next = bubble->_next;
	bubble->_next->_previous = bubble->_previous;
      }
    }
    m_bubble_factory.Delete(bubble);
  }
  
  
  void BubbleList::
  RemoveAll()
  {
    Bubble *current(m_head);
    m_head = 0;
    m_tail = 0;
    while(current != 0){
      Bubble *tmp(current->_next);
      m_bubble_factory.Delete(current);
      current = tmp;
    }
  }
  
  
  bool BubbleList::
  Empty()
    const
  {
    return ( m_head == 0) || ( m_head->Next() == 0);
  }
  
  
  void BubbleList::
  UpdateBubbles(boost::shared_ptr<const Scan> scan)
  {
    if(m_head == 0)
      return;
    
    for(Bubble *b(m_head); b != 0; b = b->_next)
      b->ResetInternalParameters();
    
    Bubble *bubble(m_head);
    while(bubble != 0){
      bubble->UpdateInternalParameters();
      bubble->UpdateExternalParameters(scan, m_bubble_band.ignore_radius);
      bubble = bubble->_next;
    }
    
    bubble = m_head;
    while(bubble != 0){
      bubble->ApplyForces();
      bubble = bubble->_next;
    }
    
    // here, the internal and external parameters do
    // not correspond to the new bubble positions, but
    // that's only a minor esthetic issue in the simulator.
    bubble = m_head;
    while(bubble != 0){
      //    bubble->UpdateInternalParameters();
      bubble->UpdateExternalParameters(scan, m_bubble_band.ignore_radius);
      bubble = bubble->_next;
    }
  }
  
  
  void BubbleList::
  UpdatePathLength()
  {
    Bubble *bubble(m_head);
    bubble->_path_length = 0;
    double dcumul(0);
    
    bubble = bubble->_next;
    while(bubble != 0){
      dcumul += bubble->_dprevious;
      bubble->_path_length = dcumul;
      bubble = bubble->_next;
    }
  }
  
  
  void BubbleList::
  UpdateBubbleBehaviour()
  {
    if(m_head == 0)
      return;
    
    for(Bubble *b(m_head); b != 0; b = b->_next){
      const double
	alpha(boundval(0.0,
		       (b->_path_length - m_shortpath) / m_deltapath,
		       1.0));
      b->SetIgnoreDistance(alpha * m_maxignoredistance);
    }
  }
  
  
  bool BubbleList::
  CheckRemove(Bubble *bubble)
  {
    if(bubble == m_tail)
      return false;
    
    Bubble *b2(bubble->_next);
    if(b2 == m_tail)
      return false;
    
    if(Bubble::CheckOverlap(*bubble, *b2->_next,
			    m_bubble_band.deletion_diameter)){
      Remove(b2);
      bubble->_dnext = Bubble::Distance(*bubble, *bubble->_next);
      bubble->_next->_dprevious = bubble->_dnext;
      return true;
    }
    
    return false;
  }
  
  
  bool BubbleList::
  CheckAdd(Bubble *bubble,
	   boost::shared_ptr<const Scan> scan)
  {
    if(bubble == m_tail)
      return false;
    
    double dx, dy, distance, radial, normal;
    int overlapType;
    
    if(Bubble::InformativeCheckOverlap(*bubble, *bubble->_next,
				       m_bubble_band.addition_diameter,
				       dx, dy, distance, radial, normal,
				       overlapType))
      return false;
    
    double rel;
    if(overlapType != Bubble::NOOVERLAP)
      rel = radial / distance;
    else
      rel = 0.5 + 0.5 * (bubble->_radius - bubble->_next->_radius) / distance;
    
    Bubble *newBubble(m_bubble_factory.Clone(bubble));
    
    if(newBubble == 0)
      return false;
    
    newBubble->UpdateExternalParameters(scan, m_bubble_band.ignore_radius);
    
    InsertAfter(bubble, newBubble);
    
    bubble->_dnext = bubble->_dnext * rel;
    newBubble->_dprevious = bubble->_dnext;
    
    newBubble->_dnext = (1 - rel) * newBubble->_next->_dprevious;
    newBubble->_next->_dprevious = newBubble->_dnext;
    
    return true;
  }
  
}
