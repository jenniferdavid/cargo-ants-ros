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


#include "HeadingObjective.hpp"
#include "DynamicWindow.hpp"
#include <cmath>


using namespace std;


namespace sfl {


  HeadingObjective::
  HeadingObjective(const DynamicWindow & dynamic_window,
		   const RobotModel & robot_model)
    : Objective(dynamic_window),
      local_goal_x(0),
      local_goal_y(0),
      angle_offset(0),
      m_robot_model(robot_model),
      m_standstill_prediction(dimension, dimension)
  {
  }
  
  
  void HeadingObjective::
  Calculate(double timestep, size_t qdlMin, size_t qdlMax,
	    size_t qdrMin, size_t qdrMax,
	    double carrot_lx, double carrot_ly,
	    boost::shared_ptr<const Scan> local_scan)
  {
    local_goal_x = carrot_lx;
    local_goal_y = carrot_ly;
    for(size_t iqdl(qdlMin); iqdl <= qdlMax; iqdl++)
      for(size_t iqdr(qdrMin); iqdr <= qdrMax; iqdr++)
	if(m_dynamic_window.Admissible(iqdl, iqdr)){
	  double sx, sy, stheta;
	  m_robot_model.PredictStandstillAct(m_dynamic_window.Qd(iqdl),
					     m_dynamic_window.Qd(iqdr),
					     timestep, sx, sy, stheta);
	  m_standstill_prediction[iqdl][iqdr].Set(sx, sy, stheta);
	  double dx(local_goal_x);
	  double dy(local_goal_y);
	  m_standstill_prediction[iqdl][iqdr].From(dx, dy);
	  const double dtheta(mod2pi(atan2(dy, dx) + angle_offset));
	  m_value[iqdl][iqdr] = 1 - absval(dtheta) / M_PI;
	}
	else
	  m_value[iqdl][iqdr] = minValue;
  }
  
  
  const Frame & HeadingObjective::
  PredictedStandstill(size_t iqdl, size_t iqdr) const
  {
    return m_standstill_prediction[iqdl][iqdr];
  }
  
}
