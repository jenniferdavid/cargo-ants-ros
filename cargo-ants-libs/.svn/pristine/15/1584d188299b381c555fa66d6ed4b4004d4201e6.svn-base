/* 
 * Copyright (C) 2009 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#include "DistanceObjectiveToBI.hpp"
#include "DynamicWindow.hpp"

using namespace boost;
using namespace std;


namespace sfl {
  
  
  DistanceObjectiveToBI::
  DistanceObjectiveToBI(const DynamicWindow & dynamic_window,
			boost::shared_ptr<const RobotModel> robot_model,
			double grid_width,
			double grid_height,
			double grid_resolution,
			ssize_t _blur_radius)
    : DistanceObjective(dynamic_window, robot_model, grid_width, grid_height, grid_resolution),
      blur_radius(_blur_radius)
  {
  }
  
  
  void DistanceObjectiveToBI::
  Initialize(std::ostream * progress_stream)
  {
    DistanceObjective::Initialize(progress_stream);
    
    // Make the blur mask bigger than the actual DWA velocity space in
    // order to not worry about border checks.
    m_blur.resize(-blur_radius, dimension + blur_radius + 1,
		  -blur_radius, dimension + blur_radius + 1);
    
    if (progress_stream)
      *progress_stream << "sfl::DistanceObjectiveToBI::Initialize(): computing blur kernel\n"
		       << "  blur_radius = " << blur_radius << "\n";
    
    // The blur kernel is a mask of lower bounds on distance objective
    // functions, centered on non-admissible speeds.
    m_blur_kernel.resize(-blur_radius, blur_radius + 1,
			 -blur_radius, blur_radius + 1);
    for (ssize_t ix(-blur_radius); ix <= blur_radius; ++ix)
      for (ssize_t iy(-blur_radius); iy <= blur_radius; ++iy) {
	// relative distance from cell to kernel center
	double const rr(sqrt(pow(ix, 2.0) + pow(iy, 2.0)) / blur_radius);
	// The blur value will be used as an upper bound for objective
	// function close to forbidden velocities. This upper bound
	// should thus be 1 when we reach (or exceed) blur_radius, and
	// it should be close to zero in the vicinity of the forbidden
	// velocity.
	//
	// HACKS:
	// - DistanceObjective::Admissible() checks against
	//   minValue+epsilon, so here we use 2*epsilon as a minimum
	//   for the lower bound.
	// - We can skip checking for rr<=1 because we never get
	//   values above MaxValue anyway.
	//
	// XXXX ToBI: play around with this, it should be a function
	// bound to [minValue, maxValue] that is "just above" minValue
	// for rr=0 and at maxValue for rr=1.
	m_blur_kernel.at(ix, iy) = 2 * epsilon + 1 - pow(maxval(0.0, 1 - rr), 4);
	//	    minValue + 2 * epsilon + (maxValue - minValue) * pow(rr, 0.75);
	if (progress_stream)
	  *progress_stream << "  rr = " << rr << "\tbound = " << m_blur_kernel.at(ix, iy) << "\n";
      }
    
    if (progress_stream)
      *progress_stream << "sfl::DistanceObjectiveToBI::Initialize(): FINISHED\n";
  }
  
  
  void DistanceObjectiveToBI::
  Calculate(double timestep, size_t qdlMin, size_t qdlMax,
	    size_t qdrMin, size_t qdrMax,
	    double carrot_lx, double carrot_ly,
	    boost::shared_ptr<const Scan> local_scan)
  {
    // It's not really required to reset the blur over the entire
    // speed space because DWA does not look outside the valid range
    // anyway. But when visualizing the DistanceObjectiveToBI values
    // it is very confusing to see stale value lying around, and
    // resetting the blur is pretty cheap compared with what other
    // operations we perform during update.
    for (ssize_t iqdl(0); iqdl < static_cast<ssize_t>(dimension); ++iqdl)
      for (ssize_t iqdr(0); iqdr < static_cast<ssize_t>(dimension); ++iqdr)      
	m_blur.at(iqdl, iqdr) = maxValue;
    
    ResetGrid();
    UpdateGrid(local_scan);
    for (ssize_t iqdl(0); iqdl < static_cast<ssize_t>(dimension); ++iqdl)
      for (ssize_t iqdr(0); iqdr < static_cast<ssize_t>(dimension); ++iqdr)      
	if ( ! m_dynamic_window.Forbidden(iqdl, iqdr)) {
	  double const ctime(MinTime(iqdl, iqdr));
	  if ((ctime == invalidTime) || (ctime >= m_max_brake_time))
	    m_value[iqdl][iqdr] = maxValue;
	  else {
	    /** \todo XXXX hmm, adding timestep to m_base_brake_time
		seemed prudent at one point, but further down we
		divide by (m_max_brake_time-btime), which might thus
		become -timestep... */
	    double const btime(m_base_brake_time[iqdl][iqdr] + timestep);
	    if (ctime <= btime) { // would be a collision!
	      m_value[iqdl][iqdr] = minValue;
	      // Apply the blur kernel, which will force its lower
	      // bound further down (after the enclosing for-loops.
	      for (ssize_t ix(m_blur_kernel.xbegin()); ix < m_blur_kernel.xend(); ++ix)
		for (ssize_t iy(m_blur_kernel.ybegin()); iy < m_blur_kernel.yend(); ++iy) {
		  ssize_t const jl(iqdl + ix);
		  ssize_t const jr(iqdr + iy);
		  m_blur.at(jl, jr) = minval(m_blur.at(jl, jr), m_blur_kernel.at(ix, iy));
		}
	    }
	    else {		// ctime > btime: no collision
	      /** \todo XXXX hang on a sec... btime is initialized
		  with foo+timestep, so conceivably we could get
		  negative objective values here! On the other hand,
		  that only happens when we're within one timestep of
		  colliding, for small timesteps that's probably
		  OK. */
	      m_value[iqdl][iqdr] = 
 		minValue
 		+ (ctime - btime) * (maxValue - minValue)
 		/ (m_max_brake_time - btime);
	    }
	  }
	}
    
    // Apply the blur we just computed... again, there is actually no
    // need to go over the entire speed space, DWA just looks inside
    // the [qdxMin, qdqdxMax] range, but let's update all of it for
    // debugging sake.
    for (ssize_t iqdl(0); iqdl < static_cast<ssize_t>(dimension); ++iqdl)
      for (ssize_t iqdr(0); iqdr < static_cast<ssize_t>(dimension); ++iqdr)      
	m_value[iqdl][iqdr] = minval(m_value[iqdl][iqdr], m_blur.at(iqdl, iqdr));
  }
  
}
