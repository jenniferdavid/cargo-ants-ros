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


#include "SpeedObjective.hpp"
#include "DynamicWindow.hpp"
#include "../api/RobotModel.hpp"


using namespace std;


namespace sfl {
  
  
  SpeedObjective::
  SpeedObjective(const DynamicWindow & dynamic_window,
		 const RobotModel & robot_model)
    : Objective(dynamic_window),
      sdMax(robot_model.SdMax()),
      m_robot_model(robot_model),
      m_forward(dimension, dimension),
      m_backward(dimension, dimension),
      m_slow(dimension, dimension),
      m_strict_forward(dimension, dimension),
      m_strict_backward(dimension, dimension),
      m_strict_slow(dimension, dimension),
      m_current( & m_forward),
      m_goForward(true)
  {
  }
  
  
  void SpeedObjective::
  Initialize(ostream * progress_stream)
  {
    for(size_t iqdr(0); iqdr < dimension; iqdr++){
      if((iqdr % 2) != 0){
	double sd, thetad;
	m_robot_model.Actuator2Global(m_dynamic_window.Qd(0),
				      m_dynamic_window.Qd(iqdr),
				      sd, thetad);
	m_forward[0][iqdr] =
	  minValue + (maxValue - minValue) * (sd + sdMax) / (2 * sdMax);
	if(sd >= 0)
	  m_strict_forward[0][iqdr] = m_forward[0][iqdr];
	else
	  m_strict_forward[0][iqdr] = minValue;	  
      }
      for(size_t iqdl(iqdr % 2); iqdl < dimension; iqdl += 2){
	double sd, thetad;
	m_robot_model.Actuator2Global(m_dynamic_window.Qd(iqdl),
				      m_dynamic_window.Qd(iqdr),
				      sd, thetad);
	const double
	  val(minValue + (maxValue - minValue) * (sd + sdMax) / (2 * sdMax));
	m_forward[iqdl][iqdr] = val;
	if(sd >= 0)
	  m_strict_forward[iqdl][iqdr] = val;
	else
	  m_strict_forward[iqdl][iqdr] = minValue;
	if(iqdl < dimension - 1){
	  m_forward[iqdl + 1][iqdr] = val;
	  if(sd >= 0)
	    m_strict_forward[iqdl + 1][iqdr] = val;
	  else
	    m_strict_forward[iqdl + 1][iqdr] = minValue;
	}
      }
    }
    
    for(size_t iqdr(0); iqdr < dimension; iqdr++){
      if((iqdr % 2) != 0){
	double sd, thetad;
	m_robot_model.Actuator2Global(m_dynamic_window.Qd(dimension - 1),
				      m_dynamic_window.Qd(iqdr),
				      sd, thetad);
	m_backward[dimension - 1][iqdr] =
	  maxValue + minValue
	  - (maxValue - minValue) * (sd + sdMax) / (2 * sdMax);
	if(sd <= 0)
	  m_strict_backward[dimension - 1][iqdr] =
	    m_backward[dimension - 1][iqdr];
	else
	  m_strict_backward[dimension - 1][iqdr] = minValue;	  
      }
      for(size_t iqdl(iqdr % 2); iqdl < dimension; iqdl += 2){
	double sd, thetad;
	m_robot_model.Actuator2Global(m_dynamic_window.Qd(iqdl),
				      m_dynamic_window.Qd(iqdr),
				      sd, thetad);
	const double
	  val(maxValue + minValue
	      - (maxValue - minValue) * (sd + sdMax) / (2 * sdMax));
	m_backward[iqdl][iqdr] = val;
	if(sd <= 0)
	  m_strict_backward[iqdl][iqdr] = val;
	else
	  m_strict_backward[iqdl][iqdr] = minValue;	  
	if(iqdl > 0){
	  m_backward[iqdl - 1][iqdr] = val;
	  if(sd <= 0)
	    m_strict_backward[iqdl - 1][iqdr] = val;
	  else
	    m_strict_backward[iqdl - 1][iqdr] = minValue;	  
	}
      }
    }
    
    for(size_t iqdl(0); iqdl < dimension; iqdl++)
      for(size_t iqdr(0); iqdr < dimension; iqdr++){
	double sd, thetad;
	m_robot_model.Actuator2Global(m_dynamic_window.Qd(iqdl),
				      m_dynamic_window.Qd(iqdr),
				      sd, thetad);
	m_slow[iqdl][iqdr] =
	  maxValue - (maxValue - minValue) * absval(sd) / sdMax;
	if(dimension - 1 == iqdl + iqdr)
	  m_strict_slow[iqdl][iqdr] = maxValue;
	else
	  m_strict_slow[iqdl][iqdr] = minValue;	  
      }
  }
  
  
  void SpeedObjective::
  Calculate(double timestep, size_t qdlMin, size_t qdlMax,
	    size_t qdrMin, size_t qdrMax,
	    double carrot_lx, double carrot_ly,
	    boost::shared_ptr<const Scan> local_scan)
  {
    for(size_t iqdl(qdlMin); iqdl <= qdlMax; iqdl++)
      for(size_t iqdr(qdrMin); iqdr <= qdrMax; iqdr++)
	if(m_dynamic_window.Admissible(iqdl, iqdr))
	  m_value[iqdl][iqdr] = (*m_current)[iqdl][iqdr];
	else
	  m_value[iqdl][iqdr] = minValue;
  }


  void SpeedObjective::
  GoFast()
  {
    if(m_goForward)
      m_current = & m_forward;
    else
      m_current = & m_backward;
  }


  void SpeedObjective::
  GoStrictFast()
  {
    if(m_goForward)
      m_current = & m_strict_forward;
    else
      m_current = & m_strict_backward;
  }


  void SpeedObjective::
  GoSlow()
  {
    m_current = & m_slow;
  }


  void SpeedObjective::
  GoStrictSlow()
  {
    m_current = & m_strict_slow;
  }


  void SpeedObjective::
  GoForward()
  {
    m_goForward = true;
  }


  void SpeedObjective::
  GoBackward()
  {
    m_goForward = false;
  }

}
