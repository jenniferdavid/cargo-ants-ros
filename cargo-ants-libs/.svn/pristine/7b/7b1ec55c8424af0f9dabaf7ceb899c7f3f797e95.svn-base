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


#include "NF1.hpp"
#include "NF1Wave.hpp"
#include "GridFrame.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/api/Scan.hpp>
#include <cmath>


using namespace boost;


namespace sfl {
  
  
  NF1::
  NF1()
    : m_frame(new GridFrame(1)),
      m_grid(new grid_t()),
      m_wave(new NF1Wave())
  {
  }
  
  
  void NF1::
  Initialize(shared_ptr<const Scan> scan,
	     double robot_radius, double goal_radius)
  {
    m_grid.reset(new grid_t(m_grid_dimension, NF1Wave::FREE));
    const size_t nscans(scan->data.size());
    for(size_t is(0); is < nscans; ++is)
      if(scan->data[is].rho >= robot_radius){
	const scan_data & gdata(scan->data[is]);
	SetGlobalDisk(*m_frame, *m_grid, position_t(gdata.globx, gdata.globy),
		      robot_radius, NF1Wave::OBSTACLE);
      }
    SetGlobalDisk(*m_frame, *m_grid, m_goal, goal_radius, NF1Wave::FREE);
    (*m_grid)[m_goal_index] = NF1Wave::GOAL;
    SetGlobalDisk(*m_frame, *m_grid, m_home, robot_radius, NF1Wave::FREE);
  }
  
  
  void NF1::
  Calculate()
  {
    m_wave->Reset();
    m_wave->AddSeed(m_frame->GlobalIndex(m_goal));
    m_wave->Propagate(*m_grid);
  }
  
  
  bool NF1::
  ResetTrace()
  {
    m_trace = m_home_index;
    if(NF1Wave::FREE == (*m_grid)[m_trace])
      return false;
    return true;
  }
  
  
  bool NF1::
  GlobalTrace(position_t & point)
  {
    point = m_frame->GlobalPoint(m_trace);
    if(NF1Wave::GOAL == (*m_grid)[m_trace])
      return false;
    m_trace = m_wave->SmallestNeighbor(*m_grid, m_trace);
    return true;
  }
  
  
  void NF1::
  Configure(double robot_x, double robot_y,
	    double goal_x, double goal_y,
	    double grid_width, size_t grid_width_dimension)
  {
    m_goal.v0 = goal_x;
    m_goal.v1 = goal_y;
    m_home.v0 = robot_x;
    m_home.v1 = robot_y;
    if(grid_width_dimension % 2 == 0)
      ++grid_width_dimension;
    
    const double dx(goal_x - robot_x);
    const double dy(goal_y - robot_y);
    const Frame frame(robot_x, robot_y, atan2(dy, dx));
    const double delta(grid_width / grid_width_dimension);
    const double width_offset(grid_width * (0.5 - 0.5 / grid_width_dimension));
    double xm_frame(- width_offset);
    double ym_frame(- width_offset);
    frame.To(xm_frame, ym_frame);
    
    m_frame->Configure(xm_frame, ym_frame, frame.Theta(), delta);
    m_goal_index = m_frame->GlobalIndex(goal_x, goal_y);
    m_home_index = m_frame->GlobalIndex(m_home);
    m_grid_dimension.v0 =
      static_cast<size_t>(ceil((sqrt(sqr(dx)+sqr(dy)) + grid_width) / delta));
    m_grid_dimension.v1 = grid_width_dimension;
  }
  
  
  shared_ptr<const NF1::grid_t> NF1::
  GetGridLayer() const
  {
    return m_grid;
  }
    
  shared_ptr<const GridFrame> NF1::
  GetGridFrame() const
  {
    return m_frame;
  }
  
  
  /** \todo this implementation is a bit brute force... */
  void NF1::
  SetLocalDisk(GridFrame const & gframe,
	       grid_t & grid, position_t center,
	       double radius, double value)
  {
    index_t index(gframe.LocalIndex(center));
    if(grid.ValidIndex(index))
      grid[index] = value;
    const index_t
      minidx(gframe.LocalIndex(center.v0 - radius, center.v1 - radius));
    const index_t
      maxidx(gframe.LocalIndex(center.v0 + radius, center.v1 + radius));
    const double radius2(sqr(radius));
    for(ssize_t ix(minidx.v0); ix < maxidx.v0; ++ix)
      for(ssize_t iy(minidx.v1); iy < maxidx.v1; ++iy)
	if(grid.ValidIndex(ix, iy)){
	  const position_t point(gframe.LocalPoint(ix, iy) - center);
	  const double r2(sqr(point.v0) + sqr(point.v1));
	  if(r2 <= radius2)
	    grid[ix][iy] = value;
	}
  }
  
  
  void NF1::
  SetGlobalDisk(GridFrame const & gframe,
		grid_t & grid, position_t center,
		double radius, double value)
  {
    gframe.From(center.v0, center.v1);
    SetLocalDisk(gframe, grid, center, radius, value);
  }
  
}
