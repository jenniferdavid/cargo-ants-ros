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


#include "DynamicWindow.hpp"
#include "DistanceObjective.hpp"
#include "HeadingObjective.hpp"
#include "SpeedObjective.hpp"
#include "../api/RobotModel.hpp"
#include "../util/pdebug.hpp"

#include <iostream>
#include <limits>
#include <stdio.h>


using namespace boost;
using namespace std;


namespace sfl {


  DynamicWindow::
  DynamicWindow(int _dimension,
		shared_ptr<const RobotModel> robot_model)
    : dimension(_dimension),
      maxindex(_dimension - 1),
      resolution(2 * robot_model->QdMax() / _dimension),
      qddMax(robot_model->QddMax()),
      m_robot_model(robot_model),
      m_reset_entire_velocity_space(false),
      m_qd(new double[_dimension]),
      m_state(_dimension, _dimension),
      m_objective(_dimension, _dimension),
      m_qdlMin(-1),
      m_qdlMax(-1),
      m_qdrMin(-1),
      m_qdrMax(-1),
      m_qdlOpt(-1),
      m_qdrOpt(-1),
      m_compute_next_optimum(false)
  {
  }
  
  
  void DynamicWindow::
  AddObjective(boost::shared_ptr<Objective> objective,
	       double alpha)
  {
    m_objmap.insert(make_pair(objective, alpha));
    if (objective->YieldsAdmissible())
      m_admobjlist.push_back(objective);
    if (objective->UsesEntireVelocitySpace())
      m_reset_entire_velocity_space = true;
  }
  
  
  void DynamicWindow::
  Initialize(ostream * os)
  {
    for(int il = 0; il < dimension; ++il)
      for(int ir = 0; ir < dimension; ++ir)
	m_state[il][ir] = REACHABLE; // make sure we initialize with something other than FORBIDDEN
    
    for(int i = 0; i < dimension; ++i)
      m_qd[i] = FindQd(i);
    InitForbidden();
    for (objmap_t::iterator ii(m_objmap.begin()); ii != m_objmap.end(); ++ii)
      ii->first->Initialize(os);
  }


  bool DynamicWindow::
  Forbidden(int qdlIndex,
	    int qdrIndex) const
  {
    return m_state[qdlIndex][qdrIndex] == FORBIDDEN;
  }


  bool DynamicWindow::
  Admissible(int qdlIndex,
	     int qdrIndex) const
  {
    return m_state[qdlIndex][qdrIndex] == ADMISSIBLE;
  }


  bool DynamicWindow::
  Reachable(int qdlIndex,
	    int qdrIndex) const
  {
    return m_state[qdlIndex][qdrIndex] == REACHABLE;
  }


  void DynamicWindow::
  Update(double qdl, double qdr, double timestep, double dx, double dy,
	 boost::shared_ptr<const Scan> local_scan,
	 ostream * dbgos)
  {
    PVDEBUG("DWA dt: %g   goal: %g   %g   qd: %g   %g\n",
	    timestep, dx, dy, qdl, qdr);
    
    CalculateReachable(timestep, qdl, qdr);
    
    if (m_admobjlist.empty())
      PDEBUG_OUT("WARNING no objective that yields info about admissible velocities\n");
    for (admobjlist_t::iterator ii(m_admobjlist.begin()); ii != m_admobjlist.end(); ++ii)
      (*ii)->Calculate(timestep, m_qdlMin, m_qdlMax, m_qdrMin, m_qdrMax,
		       dx, dy, local_scan);
    CalculateAdmissible();
    
    for (objmap_t::iterator ii(m_objmap.begin()); ii != m_objmap.end(); ++ii)
      if ( ! ii->first->YieldsAdmissible()) // otherwise already updated above
	ii->first->Calculate(timestep, m_qdlMin, m_qdlMax, m_qdrMin, m_qdrMax,
			     dx, dy, local_scan);
    
    m_compute_next_optimum = true;
    
    if(dbgos != 0){
      (*dbgos) << "INFO from DynamicWindow::Update():\n"
	       << "  dynamic window:\n";
      DumpObjectives((*dbgos), "    ");
      (*dbgos) << "  FINISHED DynamicWindow::Update():\n";
    }
  }
  
  
  void LegacyDynamicWindow::
  SetHeadingOffset(double angle)
  {
    m_heading_objective->angle_offset = angle;
  }


  double LegacyDynamicWindow::
  GetHeadingOffset() const
  {
    return m_heading_objective->angle_offset;
  }
  
  
  void LegacyDynamicWindow::
  GoFast()
  {
    PDEBUG("going fast\n");
    m_speed_objective->GoFast();
  }
  
  
  void LegacyDynamicWindow::
  _GoStrictFast()
  {
    PDEBUG("going fast, but strictly, whatever that means\n");
    m_speed_objective->GoStrictFast();
  }


  void LegacyDynamicWindow::
  GoSlow()
  {
    PDEBUG("going slow\n");
    m_speed_objective->GoSlow();
  }


  void LegacyDynamicWindow::
  _GoStrictSlow()
  {
    PDEBUG("going slow, but strictly, whatever that means\n");
    m_speed_objective->GoStrictSlow();
  }


  void LegacyDynamicWindow::
  GoForward()
  {
    PDEBUG("going forward\n");
    m_heading_objective->angle_offset = 0;
    m_speed_objective->GoForward();
  }


  void LegacyDynamicWindow::
  GoBackward()
  {
    PDEBUG("going backward\n");
    m_heading_objective->angle_offset = M_PI;
    m_speed_objective->GoBackward();
  }


  bool DynamicWindow::
  OptimalActuators(double & qdl, double & qdr) const
  {
    if(m_compute_next_optimum){
      CalculateOptimum();
      m_compute_next_optimum = false;
    }
    if((m_qdlOpt < 0) || (m_qdrOpt < 0))
      return false;
    qdl = m_qd[m_qdlOpt];
    qdr = m_qd[m_qdrOpt];
    return true;
  }


  int DynamicWindow::
  FindIndex(double qd) const
  {
    return (int) floor(0.5 * ((double) dimension) + qd / resolution);
  }


  double DynamicWindow::
  FindQd(int index) const
  {
    return resolution * (((double) index) - 0.5 * ((double) dimension - 1));
  }


  void DynamicWindow::
  InitForbidden()
  {
    const double sd_max(m_robot_model->SdMax());
    const double thetad_max(m_robot_model->ThetadMax());
    for(int il = 0; il < dimension; ++il)
      for(int ir = 0; ir < dimension; ++ir){
	double sd, thetad;
	m_robot_model->Actuator2Global(m_qd[il], m_qd[ir], sd, thetad);
	if((absval(sd) > sd_max) || (absval(thetad) > thetad_max))
	  m_state[il][ir] = FORBIDDEN;
      }
  }


  void DynamicWindow::
  CalculateReachable(double timestep, double qdl, double qdr)
  {
    const double reachableQd(timestep * qddMax);
    m_qdlMin = boundval(0, FindIndex(qdl - reachableQd), maxindex);
    m_qdlMax = boundval(0, FindIndex(qdl + reachableQd), maxindex);
    m_qdrMin = boundval(0, FindIndex(qdr - reachableQd), maxindex);
    m_qdrMax = boundval(0, FindIndex(qdr + reachableQd), maxindex);
    for(int il = m_qdlMin; il <= m_qdlMax; ++il)
      for(int ir = m_qdrMin; ir <= m_qdrMax; ++ir)
	if(m_state[il][ir] != FORBIDDEN)
	  m_state[il][ir] = REACHABLE;  
  }


  void DynamicWindow::
  CalculateAdmissible()
  {
    int qdlmin(m_qdlMin);
    int qdlmax(m_qdlMax);
    int qdrmin(m_qdrMin);
    int qdrmax(m_qdrMax);
    if (m_reset_entire_velocity_space) {
      qdlmin = 0;
      qdlmax = maxindex;
      qdrmin = 0;
      qdrmax = maxindex;
    }
    for (int il(qdlmin); il <= qdlmax; ++il)
      for (int ir(qdrmin); ir <= qdrmax; ++ir)
	if (FORBIDDEN != m_state[il][ir]) {
	  bool adm(true);
	  for (admobjlist_t::iterator ii(m_admobjlist.begin()); ii != m_admobjlist.end(); ++ii) {
	    if ( ! (*ii)->Admissible(il, ir)) {
	      adm = false;
	      break;
	    }
	  }
	  if (adm)
	    m_state[il][ir] = ADMISSIBLE;
	}
  }
  
  
  void DynamicWindow::
  CalculateOptimum() const
  {
    m_qdlOpt = -1;
    m_qdrOpt = -1;
    m_objectiveMax = numeric_limits<double>::min();
    m_objectiveMin = numeric_limits<double>::max();

    for(int il = m_qdlMin; il <= m_qdlMax; ++il)
      for(int ir = m_qdrMin; ir <= m_qdrMax; ++ir)
	if(m_state[il][ir] == ADMISSIBLE){
	  double obj(0);
	  for (objmap_t::const_iterator ii(m_objmap.begin()); ii != m_objmap.end(); ++ii)
	    obj += ii->second * ii->first->Value(il, ir);
	  m_objective[il][ir] = obj;

	  if(obj > m_objectiveMax){
	    m_objectiveMax = obj;
	    m_qdlOpt = il;
	    m_qdrOpt = ir;
	  }

	  if(obj < m_objectiveMin)
	    m_objectiveMin = obj;
	}
    
    PDEBUG("DWA [%d   %d]: %g\n", m_qdlOpt, m_qdrOpt, m_objectiveMax);
  }
  
  
  void LegacyDynamicWindow::
  DumpObstacles(ostream & os,
		const char * prefix) const
  {
    m_distance_objective->DumpGrid(os, prefix);
  }


  void DynamicWindow::
  DumpObjectives(ostream & os,
		 const char * prefix) const
  {
    static const char forbidden_char('.');
    static const char nonforbidden_char(' ');
    static const char optimum_char('O');
    static const char admissible_char(':');
    static const char collision_char('*');
    
    for(int iqdr(dimension - 1); iqdr > m_qdrMax; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < dimension; ++iqdl)
	if(m_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                 os << nonforbidden_char;
      os << "\n";
    }
    for(int iqdr(m_qdrMax); iqdr >= m_qdrMin; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < m_qdlMin; ++iqdl)
	if(m_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                 os << nonforbidden_char;
      for(int iqdl(m_qdlMin); iqdl <= m_qdlMax; ++iqdl){
	if((iqdl == m_qdlOpt) && (iqdr == m_qdrOpt)) os << optimum_char;
	else if(m_state[iqdl][iqdr] == FORBIDDEN)    os << forbidden_char;
	else if(m_state[iqdl][iqdr] == ADMISSIBLE)   os << admissible_char;
	else                                         os << collision_char;
      }
      for(int iqdl(m_qdlMax + 1); iqdl < dimension; ++iqdl)
	if(m_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                 os << nonforbidden_char;
      os << "\n";
    }
    for(int iqdr(m_qdrMin - 1); iqdr >= 0; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < dimension; ++iqdl)
	if(m_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                 os << nonforbidden_char;
      os << "\n";
    }
  }
  
  
  LegacyDynamicWindow::
  LegacyDynamicWindow(int dimension,
		      double grid_width,
		      double grid_height,
		      double grid_resolution,
		      shared_ptr<const RobotModel> robot_model,
		      double alpha_distance,
		      double alpha_heading,
		      double alpha_speed,
		      bool auto_init)
    : DynamicWindow(dimension, robot_model),
      m_distance_objective(new DistanceObjective(*this,
						 robot_model,
						 grid_width,
						 grid_height,
						 grid_resolution)),
      m_heading_objective(new HeadingObjective(*this, *robot_model)),
      m_speed_objective(new SpeedObjective(*this, *robot_model))
  {
    AddObjective(m_distance_objective, alpha_distance);
    AddObjective(m_heading_objective,  alpha_heading);
    AddObjective(m_speed_objective,    alpha_speed);
    if (auto_init)
      Initialize(&cerr);
  }
  
  
  void LegacyDynamicWindow::
  Update(double qdl,
	 double qdr,
	 double timestep,
	 double dx,
	 double dy,
	 boost::shared_ptr<const Scan> local_scan,
	 std::ostream * dbgos)
  {
    DynamicWindow::Update(qdl, qdr, timestep, dx, dy, local_scan, dbgos);
    if(dbgos != 0){
      (*dbgos) << "INFO from LegacyDynamicWindow::Update():\n"
	       << "  obstacles:\n";
      DumpObstacles((*dbgos), "    ");
      (*dbgos) << "  FINISHED LegacyDynamicWindow::Update():\n";
    }
  }
  
}
