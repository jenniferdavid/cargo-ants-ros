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


#ifndef SUNFLOWER_NF1_H
#define SUNFLOWER_NF1_H


#include <sfl/util/array2d.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {
  
  
  class Scan;
  class NF1Wave;
  class GridFrame;
  
  
  class NF1
  {
  public:
    typedef GridFrame::index_t index_t;
    typedef GridFrame::position_t position_t;

    typedef array2d<double> grid_t;
    
    NF1();
    
    void Configure(double robot_x, double robot_y,
		   double goal_x, double goal_y,
		   double grid_width, size_t grid_width_dimension);
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    void Initialize(boost::shared_ptr<const Scan> scan,
		    double robot_radius, double goal_radius);
    
    void Calculate();
    bool ResetTrace();
    bool GlobalTrace(position_t & point);
    
    /** \note for plotting. */
    boost::shared_ptr<const grid_t> GetGridLayer() const;
    
    /** \note for plotting. */
    boost::shared_ptr<const GridFrame> GetGridFrame() const;
    
    
  private:
    boost::shared_ptr<GridFrame> m_frame;
    index_t m_grid_dimension;
    position_t m_goal;
    index_t m_goal_index;
    position_t m_home;
    index_t m_home_index;
    boost::shared_ptr<grid_t> m_grid;
    boost::shared_ptr<NF1Wave> m_wave;
    index_t m_trace;
    
    
    // refactored from GridFrame
    static void SetLocalDisk(GridFrame const & gframe,
			     grid_t & grid, position_t center,
			     double radius, double value);
    
    // refactored from GridFrame
    static void SetGlobalDisk(GridFrame const & gframe,
			      grid_t & grid, position_t center,
			      double radius, double value);
  };
  
}

#endif // SUNFLOWER_NF1_H
