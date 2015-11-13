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

#ifndef SUNFLOWER_DIST_OBJ_TOBI_HPP
#define SUNFLOWER_DIST_OBJ_TOBI_HPP

#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/util/flexgrid.hpp>

namespace sfl {
  
  
  /**
     A variation of the sfl::DistanceObjective developed for the
     University of Bielefeld's entry to the RoboCup@Home competition
     in 2009. It has a more careful treatment of potential collisions,
     by looking beyond just the reachable set of actuator
     commands. This allows to "blur" velocity space obstacles that lie
     outside the currently permitted choices, in order to anticipate
     what would happen several timesteps in the future if.
     
     Results with this distance objective are a bit mixed (at the time
     of writing this comment, anyway): the robot is more cautious of
     obstacles, but sometimes to the point of getting stuck more
     easily.  But it definitely does add more fore-sightedness when
     compared to the Expo.02 robots.
   */
  class DistanceObjectiveToBI
    : public DistanceObjective
  {
  public:
    DistanceObjectiveToBI(const DynamicWindow & dynamic_window,
			  boost::shared_ptr<const RobotModel> robot_model,
			  double grid_width,
			  double grid_height,
			  double grid_resolution,
			  ssize_t blur_radius);
    
    virtual void Initialize(std::ostream * progress_stream);
    
    virtual bool UsesEntireVelocitySpace() const { return true; }
    
    virtual void Calculate(double timestep, size_t qdlMin, size_t qdlMax,
			   size_t qdrMin, size_t qdrMax,
			   double carrot_lx, double carrot_ly,
			   boost::shared_ptr<const Scan> local_scan);
    
    ssize_t const blur_radius;    
    
  protected:
    flexgrid<double> m_blur;
    flexgrid<double> m_blur_kernel;
  };

}

#endif // SUNFLOWER_DIST_OBJ_TOBI_HPP
