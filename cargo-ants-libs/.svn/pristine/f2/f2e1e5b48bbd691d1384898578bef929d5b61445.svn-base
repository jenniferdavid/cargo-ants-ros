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


#include "Odometry.hpp"
#include "Pose.hpp"
#include <iostream>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  Odometry::
  Odometry(boost::shared_ptr<sfl::LocalizationInterface> localization)
    : m_localization(localization)
  {
  }
  
  
  int Odometry::
  Update()
  {
    Pose pose;
    m_localization->GetPose(pose);
    Update(pose);
    return 0;
  }
  
  
  int Odometry::
  Update(Pose const & pose)
  {
    m_history.insert(make_pair(pose.m_tstamp, shared_ptr<Pose>(new Pose(pose))));
    return 0;
  }
  
  
  shared_ptr<const Pose> Odometry::
  Get() const
  {
    shared_ptr<Pose> pose;
    if(m_history.empty())
      pose.reset(new Pose());
    else
      pose = m_history.rbegin()->second;
    return pose;
  }
  
 
  Odometry::history_t::value_type Odometry::
  Get(const Timestamp & t) const
  {
    if(m_history.empty())
      return make_pair(Timestamp::first, shared_ptr<Pose>());
    
    history_t::const_iterator cand1(m_history.lower_bound(t));
    if(cand1 == m_history.end())
      return *m_history.rbegin();
    if((cand1->first == t) || (cand1 == m_history.begin()))
      return *cand1;
    
    history_t::const_iterator cand2(cand1--);
    Timestamp t1(t);
    t1 -= cand1->first;
    Timestamp t2(cand2->first);
    t2 -= t;
    if(t1 < t2)
      return *cand1;
    return *cand2;
  }
  
}
