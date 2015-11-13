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


#include "DistanceObjective.hpp"
#include "DynamicWindow.hpp"
#include "Lookup.hpp"
#include <sfl/util/pdebug.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdio.h>


using namespace boost;
using namespace std;


namespace sfl {


  const double DistanceObjective::invalidTime(-1);
  const size_t DistanceObjective::nearpoints_chunksize(64);
  
  
  DistanceObjective::
  DistanceObjective(const DynamicWindow & dynamic_window,
		    boost::shared_ptr<const RobotModel> robot_model,
		    double grid_width,
		    double grid_height,
		    double grid_resolution)
    : Objective(dynamic_window),
      m_qdd_max(robot_model->QdMax()),
      m_max_brake_time(robot_model->QdMax() / m_qdd_max),
      m_robot_model(robot_model),
      m_hull(robot_model->GetSafetyHull()),
      m_x0( - grid_width  / 2),
      m_y0( - grid_height / 2),
      m_x1(   grid_width  / 2),
      m_y1(   grid_height / 2),
      m_n_zone(0),
      m_point_in_hull(true),	// start with brakes on!
      m_base_brake_time(dimension, dimension, invalidTime),
      m_nearpoints(nearpoints_chunksize),
      m_n_nearpoints(0)
  {
    double delta(absval(m_x1 - m_x0));
    double dim(ceil(absval(delta / grid_resolution)));
    _dx = delta / dim;
    _dxInv = dim / delta;
    _dimx = static_cast<size_t>(dim);
    
    delta = absval(m_y1 - m_y0);
    dim = ceil(absval(delta / grid_resolution));
    _dy = delta / dim;
    _dyInv = dim / delta;
    _dimy = static_cast<size_t>(dim);
    
    m_grid.reset(new array2d<bool>(_dimx, _dimy, false));
    m_region.reset(new array2d<short>(_dimx, _dimy, NONE));
    m_time_lookup.reset(new array2d<shared_ptr<Lookup> >(_dimx, _dimy));
    
    // determine padded hull
    double padding(sqrt(_dx * _dx + _dy * _dy));
    m_padded_hull = m_hull->CreateGrownHull(padding);
    
    // one day maybe this will become more than just a bbox...
    Polygon foo;
    foo.AddPoint(m_x0, m_y0);
    foo.AddPoint(m_x1, m_y0);
    foo.AddPoint(m_x1, m_y1);
    foo.AddPoint(m_x0, m_y1);
    m_evaluation_hull.reset(new Hull(foo));
  }
  
  
  void DistanceObjective::
  Initialize(ostream * progress_stream)
  {
    // precalculate lookup tables
    m_qd_lookup.clear();
    for(size_t ii(0); ii < dimension; ++ii)
      m_qd_lookup.push_back(m_dynamic_window.Qd(ii));
    
    for(size_t ii(0); ii < dimension; ++ii)
      for(size_t jj(0); jj < dimension; ++jj)
	m_base_brake_time[ii][jj] =
	  maxval(absval(m_qd_lookup[ii]), absval(m_qd_lookup[jj])) / m_qdd_max;
    
    if(progress_stream != 0)
      (*progress_stream) << "DistanceObjective::Initialize()\n"
			 << "   calculating main lookup table...\n"
			 << flush;
    
    for(ssize_t igy(_dimy - 1); igy >= 0; --igy){
      const double yy(FindYlength(igy));
      for(size_t igx(0); igx < _dimx; ++igx){
	const double xx(FindXlength(igx));
	if( ! m_evaluation_hull->Contains(xx, yy))
	  continue;
	if(m_padded_hull->Contains(xx, yy)){
	  if(progress_stream != 0) (*progress_stream) << "." << flush;
	  (*m_region)[igx][igy] = HULL;
	}
	else{
	  array2d<double> buffer(dimension, dimension, invalidTime);
	  size_t nValidCollisions(0);
	  
	  // fill the actuator lookup table with collision times
	  for(size_t iqdl(0); iqdl < dimension; ++iqdl)
	    for(size_t iqdr(0); iqdr < dimension; ++iqdr)
	      if( ! m_dynamic_window.Forbidden(iqdl, iqdr)){
		const double tt(PredictCollision(*m_padded_hull,
						 m_qd_lookup[iqdl],
						 m_qd_lookup[iqdr],
						 xx, yy));
		if((tt > 0) && (tt <= m_max_brake_time)){
		  ++nValidCollisions;
		  buffer[iqdl][iqdr] = tt;
		}
	      }
	  if(nValidCollisions > 0){
	    if(progress_stream != 0) (*progress_stream) << "*" << flush;
	    (*m_region)[igx][igy] = ZONE;
	    (*m_time_lookup)[igx][igy].
	      reset(new Lookup(buffer, 0, m_max_brake_time, invalidTime));
	  }
	  else{
	    if(progress_stream != 0) (*progress_stream) << "o" << flush;
	    (*m_region)[igx][igy] = NONE;
	  }
	}
      } // for igx
      if(progress_stream != 0) (*progress_stream) << "\n" << flush;
    } // for igy
    if(progress_stream != 0) (*progress_stream) << "   finished.\n" << flush;
  }
  
  
  bool DistanceObjective::
  CheckLookup(ostream * os) const
  {
    if(0 != os)
      (*os) << "INFO from DistanceObjective::CheckLookup():\n";
    for(size_t ii(0); ii < dimension; ++ii)
      if(m_qd_lookup[ii] != m_dynamic_window.Qd(ii)){
	if(0 != os)
	  (*os) << "  ERROR m_qd_lookup[" << ii << "] is " << m_qd_lookup[ii]
		<< " but should be " << m_dynamic_window.Qd(ii) << "\n";
	return false;
      }
    
    for(size_t ii(0); ii < dimension; ++ii)
      for(size_t jj(0); jj < dimension; ++jj){
	const double check(maxval(absval(m_qd_lookup[ii]),
				  absval(m_qd_lookup[jj]))
			   / m_qdd_max);
	if(epsilon < absval(m_base_brake_time[ii][jj] - check)){
	  if(0 != os)
	    (*os) << "  ERROR m_base_brake_time[" << ii << "][" << jj
		  << "] is " << m_base_brake_time[ii][jj] << "but should be "
		  << check << "\n        difference = "
		  << m_base_brake_time[ii][jj] - check << "\n";
	  return false;
	}
      }
    
    for(ssize_t igy(_dimy - 1); igy >= 0; --igy){
      if(0 != os)
	(*os) << "  ";
      double yy(FindYlength(igy));
      for(size_t igx(0); igx < _dimx; ++igx){
	double xx(FindXlength(igx));
	for(size_t iqdl(0); iqdl < dimension; ++iqdl){
	  for(size_t iqdr(0); iqdr < dimension; ++iqdr){
	    double wanted(invalidTime);
	    if(m_evaluation_hull->Contains(xx, yy)) {
	      if(m_padded_hull->Contains(xx, yy))
		wanted = epsilon; // due to epsilon hack above...
	      else{
		if( ! m_dynamic_window.Forbidden(iqdl, iqdr)){
		  const double tt(PredictCollision(*m_padded_hull,
						   m_qd_lookup[iqdl],
						   m_qd_lookup[iqdr],
						   xx, yy));
		  if((tt > 0) && (tt <= m_max_brake_time))
		    wanted = tt;
		}
	      }
	    }
	    double compressed(invalidTime);
	    if((*m_time_lookup)[igx][igy])
	      compressed = (*m_time_lookup)[igx][igy]->Get(iqdl, iqdr);
	    
	    if(wanted != compressed){
	      if(invalidTime != wanted){
		if((*m_time_lookup)[igx][igy]){
		  if(0 != os)
		    (*os) << "\n  ERROR (*m_time_lookup)["
			  << igx << "][" << igy
			  << "] should be invalidTime but is " << compressed
			  << "\n  cell [" << igx << "][" << igy << "] at ("
			  << xx << ", " << yy << ")\n";
		  return false;
		}
		if(0 != os)
		  (*os) << "\nBUG in check procedure?\n";
		return false;
	      }
	      if(invalidTime != compressed){
		if(0 != os)
		  (*os) << "\n  ERROR (*m_time_lookup)[" << igx << "][" << igy
			<< "] should be " << wanted << " but is "
			<< compressed << " (which should be invalidTime)\n"
			<< "  cell [" << igx << "][" << igy << "] at (" << xx
			<< ", " << yy << ")\n";
		return false;
	      }
	    }
	    // update stats here?
	  }
	}
	if(0 != os)
	  (*os) << ".";
      }
      if(0 != os)
	(*os) << "\n";
    }
    return true;
  }
  
  
  void DistanceObjective::
  Calculate(double timestep, size_t qdlMin, size_t qdlMax, size_t qdrMin, size_t qdrMax,
	    double carrot_lx, double carrot_ly, shared_ptr<const Scan> local_scan)
  {
    ResetGrid();
    UpdateGrid(local_scan);
    for(size_t iqdl(qdlMin); iqdl <= qdlMax; ++iqdl)
      for(size_t iqdr(qdrMin); iqdr <= qdrMax; ++iqdr)      
	if( ! m_dynamic_window.Forbidden(iqdl, iqdr)){
	  const double ctime(MinTime(iqdl, iqdr));
	  if((ctime == invalidTime) || (ctime >= m_max_brake_time))
	    m_value[iqdl][iqdr] = maxValue;
	  else{
	    const double btime(m_base_brake_time[iqdl][iqdr] + timestep);
	    if(ctime <= btime)
	      m_value[iqdl][iqdr] = minValue;
	    else
	      m_value[iqdl][iqdr] =
		minValue
		+ (ctime - btime) * (maxValue - minValue)
		/ (m_max_brake_time - btime);
	  }
	}
  }
  
  
  void DistanceObjective::
  ResetGrid()
  {
    m_n_zone = 0;
    m_point_in_hull = false;
    m_n_nearpoints = 0;
    for(size_t ix(0); ix < _dimx; ++ix)
      for(size_t iy(0); iy < _dimy; ++iy)
	(*m_grid)[ix][iy] = false;
  }
  
  
  void DistanceObjective::
  UpdateGrid(shared_ptr<const Scan> local_scan)
  {
    const size_t nscans(local_scan->data.size());
    for(size_t is(0); is < nscans; ++is){
      const scan_data & ldata(local_scan->data[is]);
      if(m_evaluation_hull->Contains(ldata.locx, ldata.locy)){
	const ssize_t ix(FindXindex(ldata.locx));
	if((0 > ix) || (_dimx <= static_cast<size_t>(ix)))
	  continue;
	const ssize_t iy(FindYindex(ldata.locy));
	if((0 > iy) || (_dimy <= static_cast<size_t>(iy)))
	  continue;
	switch((*m_region)[ix][iy]){
	case NONE:
	  (*m_grid)[ix][iy] = true; // really only for plotting...
	  break;
	case ZONE:
	  if(false == (*m_grid)[ix][iy]){ // don't count cells more than once
	    ++m_n_zone;
	    (*m_grid)[ix][iy] = true;
	  }
	  break;
	case HULL:
	  if(m_hull->Contains(ldata.locx, ldata.locy))
	    m_point_in_hull = true;
	  else{	  // will need to precisely predict collision later...
	    if(m_nearpoints.size() <= m_n_nearpoints)
	      m_nearpoints.resize(m_nearpoints.size() + nearpoints_chunksize);
	    m_nearpoints[m_n_nearpoints].v0 = ldata.locx;
	    m_nearpoints[m_n_nearpoints].v1 = ldata.locy;
	    ++m_n_nearpoints;
	  }
	  break;
	default:
	  PDEBUG_ERR("BUG in sfl::DistanceObjective::UpdateGrid(): "
		     "invalid region[%zd][%zd]=%d\n",
		     ix, iy, (*m_region)[ix][iy]);
	}
      }
    }
  }
  
  
  double DistanceObjective::
  MinTime(size_t iqdl, size_t iqdr)
  {
    if(m_point_in_hull){
      PVDEBUG("shortcut: point in hull\n");
      return epsilon;		// old hack epsilon...
    }
    
    // precisely predict collisions with "really dangerously close" points
    double minTime(invalidTime);
    for(size_t ii(0); ii < m_n_nearpoints; ++ii){
      const double tt(PredictCollision(*m_hull,
				       m_qd_lookup[iqdl],
				       m_qd_lookup[iqdr],
				       m_nearpoints[ii].v0,
				       m_nearpoints[ii].v1));
      if((tt != invalidTime)
	 && ((tt < minTime) || (minTime == invalidTime)))
	minTime = tt;
    }
    
    // use precalculated lookup tables for points that are farther away
    if(0 == m_n_zone){
      PVDEBUG("shortcut: no points in zone\n");
      return minTime;
    }
    size_t n_zone(0);
    for(size_t ix(0); ix < _dimx; ++ix)
      for(size_t iy(0); iy < _dimy; ++iy)
	if((*m_grid)[ix][iy]
	   && (ZONE == (*m_region)[ix][iy])){
	  ++n_zone;
	  const shared_ptr<const Lookup> lkup((*m_time_lookup)[ix][iy]);
	  if( ! lkup)
	    PDEBUG_ERR("BUG in sfl::DistanceObjective::MinTime(): "
		       "no lookup for zone index [%zd][%zd]\n", ix, iy);
	  else{
	    const double tt(lkup->Get(iqdl, iqdr));
	    if((tt != invalidTime)
	       && ((tt < minTime) || (minTime == invalidTime)))
	      minTime = tt;
	  }
	  if(n_zone >= m_n_zone){
	    PVDEBUG("shortcut: %d of %d zone cells done\n", n_zone, m_n_zone);
	    return minTime;
	  }
	}
    PDEBUG("BIZARRE: didn't expect to reach this point...\n");
    return minTime;
  }
  
  
  ssize_t DistanceObjective::
  FindXindex(double d) const
  {
    return static_cast<ssize_t>(floor((d - m_x0) * _dxInv));
  }
  
  
  double DistanceObjective::
  FindXlength(ssize_t i) const
  {
    return (0.5 + (double) i) * _dx + m_x0;
  }
  
  
  ssize_t DistanceObjective::
  FindYindex(double d) const
  {
    return static_cast<ssize_t>(floor((d - m_y0) * _dyInv));
  }
  
  
  double DistanceObjective::
  FindYlength(ssize_t i) const
  {
    return (0.5 + (double) i) * _dy + m_y0;
  }
  
  
  double DistanceObjective::
  PredictCollision(const Hull & hull,
		   double qdl, double qdr, double lx, double ly) const
  {
    double tMin(invalidTime);
    double sd, thetad;
    m_robot_model->Actuator2Global(qdl, qdr, sd, thetad);
    
    if(absval(epsilon * sd) < absval(thetad / epsilon)){ // circular equation
      const double thetadInv(absval(1 / thetad));
      const double rCur(sd / thetad);
      const double rGir(sqrt(sqr(lx) + sqr(ly - rCur)));
      const double phi0(atan2((ly - rCur), lx));
      
      for(HullIterator ih(hull); ih.IsValid(); ih.Increment()){
	double qx[2], qy[2];
	bool vv[2];
	LineCircleIntersect(ih.GetX0(), ih.GetY0(),
			    ih.GetX1(), ih.GetY1(),
			    0, rCur, rGir,
			    qx[0], qy[0],
			    qx[1], qy[1],
			    vv[0], vv[1]);
	for(size_t ii(0); ii < 2; ++ii){
	  if(vv[ii]){
	    double phi(atan2(qy[ii] - rCur, qx[ii]) - phi0);
	    if(thetad >= 0) phi = mod2pi( - phi);
	    else            phi = mod2pi(   phi);
	    if(phi < 0)     phi += 2 * M_PI;
	    const double tt(phi * thetadInv);
	    if((tMin < 0) || (tt < tMin))
	      tMin = tt;
	  }
	}
      }
    }
    else if(absval(sd) > epsilon){ // straight line approximation
      const double sdInv(absval(1 / sd));
      for(HullIterator ih(hull); ih.IsValid(); ih.Increment()){
	const double tt(sdInv * LineRayIntersect(ih.GetX0(), ih.GetY0(),
						 ih.GetX1(), ih.GetY1(),
						 lx, ly,
						 sd >= 0 ? -1 : 1, 0));
	if((tt >= 0) && ((tMin < 0) || (tt < tMin)))
	  tMin = tt;
      }
    }
    // else don't touch tMin, invalidTime means "no collision"
    
    return tMin;
  }
  
  
  void DistanceObjective::
  GetRange(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = m_x0;
    y0 = m_y0;
    x1 = m_x1;
    y1 = m_y1;
  }

  
  void DistanceObjective::
  DumpGrid(ostream & os, const char * prefix) const
  {
    for(ssize_t igy(_dimy - 1); igy >= 0; --igy){
      const double yy(FindYlength(igy));
      os << prefix;
      for(size_t igx(0); igx < _dimx; ++igx){
	const double xx(FindXlength(igx));
	if(m_padded_hull->Contains(xx, yy))
	  os << ((*m_grid)[igx][igy] ? "o" : ".");
	else{
	  if((*m_grid)[igx][igy])
	    os << ((*m_time_lookup)[igx][igy] ? "x" : "o");
	  else
	    os << ((*m_time_lookup)[igx][igy] ? "-" : ".");
	}
      }
      os << "\n";
    }
  }
  
  
  double DistanceObjective::
  CollisionTime(size_t ix, size_t iy, size_t iqdl, size_t iqdr) const
  {
    if((*m_time_lookup)[ix][iy])
      return (*m_time_lookup)[ix][iy]->Get(iqdl, iqdr);
    return invalidTime;
  }
  
  
  size_t DistanceObjective::
  DimX() const
  {
    return _dimx;
  }
  
  
  size_t DistanceObjective::
  DimY() const
  {
    return _dimy;
  }
  
  
  bool DistanceObjective::
  CellOccupied(size_t ix, size_t iy) const
  {
    return (*m_grid)[ix][iy];
  }
  
  
  boost::shared_ptr<const Hull> DistanceObjective::
  GetHull() const
  {
    return m_hull;
  }
  
  
  boost::shared_ptr<const Hull> DistanceObjective::
  GetPaddedHull() const
  {
    return m_padded_hull;
  }
  
  
  boost::shared_ptr<const Hull> DistanceObjective::
  GetEvaluationHull() const
  {
    return m_evaluation_hull;
  }
  
  
  short DistanceObjective::
  GetRegion(size_t ix, size_t iy) const
  {
    return (*m_region)[ix][iy];
  }
  
  
  size_t DistanceObjective::
  GetNNear() const
  {
    return m_n_nearpoints;
  }
  
  
  bool DistanceObjective::
  GetNear(size_t index, double & lx, double & ly) const
  {
    if(index >= m_n_nearpoints)
      return false;
    lx = m_nearpoints[index].v0;
    ly = m_nearpoints[index].v1;
    return true;
  }
  
  
  double DistanceObjective::
  GetDeltaX() const
  {
    return _dx;
  }
  
  
  double DistanceObjective::
  GetDeltaY() const
  {
    return _dy;
  }
  
  
  bool DistanceObjective::
  ObstacleInHull() const
  {
    return m_point_in_hull;
  }
  
  
  bool DistanceObjective::
  Admissible(int qdlIndex, int qdrIndex) const
  {
    return m_value[qdlIndex][qdrIndex] > minValue + epsilon;
  }
  
}
