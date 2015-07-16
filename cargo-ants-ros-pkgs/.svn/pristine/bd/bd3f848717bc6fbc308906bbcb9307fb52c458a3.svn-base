/* Cargo-ANTs software prototype.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "cargo_ants_util/estar_util.hpp"
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/util/numeric.hpp>

using namespace cargo_ants_msgs;


namespace cargo_ants_util {
  
  
  DistanceTransform::
  DistanceTransform (sfl::GridFrame const & world_to_estar,
		     estar_grid_t const & grid)
    : world_to_estar_ (world_to_estar)
  {
    estar_grid_init (&estar_grid_, grid.dimx, grid.dimy);
    memcpy (estar_grid_.cell, grid.cell, grid.dimx * grid.dimy);
  }
  
  
  DistanceTransform::
  ~DistanceTransform ()
  {
    estar_grid_fini (&estar_grid_);
  }
  
  
  double DistanceTransform::
  getGoalDistance (double robotx, double roboty, int * opt_errcode)
    const
  {
    sfl::GridFrame::index_t const poseidx (world_to_estar_.GlobalIndex (robotx, roboty));
    if (poseidx.v0 < 0 || poseidx.v0 >= estar_grid_.dimx || 
	poseidx.v1 < 0 || poseidx.v1 >= estar_grid_.dimy) {
      if (opt_errcode) {
	*opt_errcode = NOT_IN_GRID;
      }
      return -1.0;
    }
    
    estar_cell_t * cell (estar_grid_at (&estar_grid_, poseidx.v0, poseidx.v1));
    if (cell->flags & ESTAR_FLAG_OBSTACLE) {
      if (opt_errcode) {
	*opt_errcode = IN_OBSTACLE;
      }
      return -1.0;
    }
    
    if (opt_errcode) {
      *opt_errcode = SUCCESS;
    }
    return cell->phi;
  }


  bool DistanceTransform::
  trace (double startx, double starty, double viadist, trace_t & trace, int * opt_errcode)
    const
  {
    double const viadist2 (pow (viadist, 2.0));
    
    sfl::GridFrame::index_t const poseidx (world_to_estar_.GlobalIndex (startx, starty));
    estar_cell_t * cell (estar_grid_at (&estar_grid_, poseidx.v0, poseidx.v1));

    if (opt_errcode) {
      *opt_errcode = 0;
    }    
    
    if (std::isinf (cell->rhs)) {
      if (opt_errcode) {
	if (std::isinf (cell->phi)) {
	  // start cell has infinite rhs and phi
	  *opt_errcode = 1;
	}
	else {
	  // start cell has infinite rhs (but finite phi)
	  *opt_errcode = 2;
	}
      }
      return false;
    }
    
    trace.clear();
    double sub_ix (poseidx.v0);
    double sub_iy (poseidx.v1);
    trace_point tp; 

    double const stepsize (world_to_estar_.Delta() / 4); // make this configurable?

    for (size_t bailout (0); bailout < 10; ++bailout) {
      if (0 == estar_cell_calc_gradient (cell, &tp.gradx, &tp.grady)) {
	if (trace.empty()) {
	  if (opt_errcode) {
	    // could not compute gradient at start
	    *opt_errcode = 3;
	  }
	  return false;
	}
      }
      double const gradmag (sqrt (pow (tp.gradx, 2) + pow (tp.grady, 2)));
      tp.gradx *= stepsize / gradmag;
      tp.grady *= stepsize / gradmag;
      sub_ix += tp.gradx;
      sub_iy += tp.grady;
      
      // (sub_ix, sub_iy) is the sub-pixel grid coordinate of where we
      // want to be next. This needs to get translated to global
      // coordinates by taking into account the resolution and origin.
      //
      // If that is farther than viadist away from the previous trace
      // point (or the start if the trace is empty) then extend the
      // trace.
      
      tp.posx = sub_ix * world_to_estar_.Delta();
      tp.posy = sub_iy * world_to_estar_.Delta();
      world_to_estar_.To (tp.posx, tp.posy);
      
      double dist2;
      if (trace.empty()) {
	dist2 = pow (tp.posx - startx, 2) + pow (tp.posy - starty, 2);
      }
      else {
	dist2 = pow (tp.posx - trace.back().posx, 2) + pow (tp.posy - trace.back().posy, 2);
      }
      
      bool appended (false);
      if (dist2 >= viadist2) {
	bailout = 0;
	trace.push_back (tp);
	appended = true;
      }
      
      ssize_t const ix ((ssize_t) rint(sub_ix));
      ssize_t const iy ((ssize_t) rint(sub_iy));
      if (ix < 0 || ix >= estar_grid_.dimx || iy < 0 || iy >= estar_grid_.dimy) {
	if (opt_errcode) {
	  // fell off the map
	  *opt_errcode = 4;
	}
	return false;
      }
      
      cell = estar_grid_at (&estar_grid_, ix, iy);
      if (cell->flags & ESTAR_FLAG_GOAL) {
	// reached the goal
	if ( ! appended) {
	  trace.push_back (tp);
	}
	return true;
      }
    }
    
    // how did we end up here?
    if (opt_errcode) {
      *opt_errcode = 5;
    }
    return false;
  }
  
  
  EstarHelper::
  EstarHelper ()
    : // map_resolution_ (1.0),
      // robot_radius_ (1.5),
      // buffer_zone_ (3.0),
      // decay_power_ (2.0),
    //      world_to_mapper2d_ (0.0, 0.0, 0.0, map_resolution_),
    world_to_estar_ (0.0, 0.0, 0.0, 1.0),
    estar_ (0)
  {
  }
  
  
  EstarHelper::
  ~EstarHelper ()
  {
    if (estar_) {
      estar_fini (estar_);
    }
    delete estar_;
  }
  
  
  void EstarHelper::
  readMap (ObstacleMap::ConstPtr const & site_map,
	   double x0, double y0, double x1, double y1,
	   double map_resolution,
	   double robot_radius,
	   double buffer_zone,
	   double decay_power)
  {
    sfl::GridFrame const world_to_mapper2d (0.0, 0.0, 0.0, map_resolution);
    boost::shared_ptr <sfl::Mapper2d>
      m2d (sfl::Mapper2d::Create (world_to_mapper2d, robot_radius, buffer_zone, decay_power));
    boost::shared_ptr <sfl::TraversabilityMap> travmap (m2d->GetTravmap());
    int const freespace (travmap->freespace);
    travmap->Autogrow (x0, y0, freespace);
    travmap->Autogrow (x1, y1, freespace);

    for (size_t io (0); io < site_map->obstacles.size(); ++io) {
      Obstacle const & obstacle (site_map->obstacles[io]);
      sfl::Frame frame (obstacle.origin.ox, obstacle.origin.oy, obstacle.origin.oth);
      for (size_t il (0); il < obstacle.polylines.size(); ++il) {
	Polyline const & polyline (obstacle.polylines[il]);
	if (polyline.points.size() == 1) {
	  double px (polyline.points[0].px);
	  double py (polyline.points[0].py);
	  frame.To (px, py);
	  m2d->AddOneGlobalObstacle (px, py, false, 0);
	}
	else {
	  double p0x (polyline.points[0].px);
	  double p0y (polyline.points[0].py);
	  frame.To (p0x, p0y);
	  for (size_t ip (1); ip < polyline.points.size(); ++ip) {
	    double p1x (polyline.points[ip].px);
	    double p1y (polyline.points[ip].py);
	    frame.To (p1x, p1y);
	    m2d->AddObstacleLine (p0x, p0y, p1x, p1y, false);
	  }
	}
      }
    }
    
    ssize_t const xbegin (travmap->grid.xbegin());
    ssize_t const ybegin (travmap->grid.ybegin());
    ssize_t const xend (travmap->grid.xend());
    ssize_t const yend (travmap->grid.yend());
    ssize_t const dimx (xend - xbegin);
    ssize_t const dimy (yend - ybegin);
    int const obstacle (travmap->obstacle);
    double const scale (1.0 / (obstacle - freespace));
    sfl::TraversabilityMap::grid_t const & grid (travmap->grid);
    
    if (estar_) {
      estar_fini (estar_);
    }
    else {
      estar_ = new estar_t();
    }
    estar_init (estar_, dimx, dimy);
    
    for (ssize_t ix (xbegin); ix < xend; ++ix) {
      for (ssize_t iy (ybegin); iy < yend; ++iy) {
	estar_set_speed (estar_, ix - xbegin, iy - ybegin,
			 sfl::boundval (0.0, scale * (obstacle - grid.at(ix, iy)), 1.0));
      }
    }
    
    sfl::GridFrame::position_t const org (world_to_mapper2d.GlobalPoint (xbegin, ybegin));
    world_to_estar_.Configure (org.v0, org.v1,
			       world_to_mapper2d.Theta(), world_to_mapper2d.Delta());
  }
  
  
  DistanceTransform * EstarHelper::
  createDistanceTransform (double goalx, double goaly, int * opt_errcode)
  {
    if ( ! estar_) {
      if (opt_errcode) {
	*opt_errcode = NO_MAP;
      }
      return 0;
    }
    
    sfl::GridFrame::index_t const goal_estar (world_to_estar_.GlobalIndex (goalx, goaly));
    if (goal_estar.v0 < 0 || goal_estar.v0 >= estar_->grid.dimx || 
	goal_estar.v1 < 0 || goal_estar.v1 >= estar_->grid.dimy) {
      if (opt_errcode) {
	*opt_errcode = NOT_IN_GRID;
      }
      return 0;
    }
    
    estar_reset (estar_);
    estar_set_goal (estar_, goal_estar.v0, goal_estar.v1);
    while (estar_->pq.len != 0) {
      estar_propagate (estar_);
    }
    
    if (opt_errcode) {
      *opt_errcode = SUCCESS;
    }
    return new DistanceTransform (world_to_estar_, estar_->grid);
  }

}
