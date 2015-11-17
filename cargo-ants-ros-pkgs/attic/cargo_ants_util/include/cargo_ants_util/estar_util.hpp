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

#ifndef CARGO_ANTS_UTIL_ESTAR_UTIL_HPP
#define CARGO_ANTS_UTIL_ESTAR_UTIL_HPP

#include <sfl/gplan/GridFrame.hpp>
#include <estar2/estar.h>
#include <cargo_ants_msgs/ObstacleMap.h>


namespace cargo_ants_util {
  
  
  class DistanceTransform
  {
  public:
    enum {
      SUCCESS,
      NOT_IN_GRID,
      IN_OBSTACLE
    };
    
    struct trace_point {
      double posx, posy, gradx, grady;
    };
    typedef std::vector <trace_point> trace_t;
    
    DistanceTransform (sfl::GridFrame const & world_to_estar,
		       estar_grid_t const & grid);
    ~DistanceTransform ();
    
    double getGoalDistance (double robotx, double roboty, int * opt_errcode = 0) const;

    bool trace (double startx, double starty, double viadist,
		trace_t & trace, int * opt_errcode = 0) const;
    
  protected:
    sfl::GridFrame const world_to_estar_;
    estar_grid_t estar_grid_;

  private:
    ros::Publisher distance_pub_;
  };
  
  
  class EstarHelper
  {
  public:
    enum {
      SUCCESS,
      NOT_IN_GRID,
      NO_MAP
    };
    
    EstarHelper ();
    ~EstarHelper ();
    
    /**
       \note The (x0, y0, x1, y1) are a bounding box for the minimum
       costmap size.  This allows to avoid problems when i.e. the
       robot drives off the map while you plan.
    */
    void readMap (cargo_ants_msgs::ObstacleMap::ConstPtr const & site_map,
		  double x0, double y0, double x1, double y1,
		  double map_resolution,
		  double robot_radius,
		  double buffer_zone,
		  double decay_power);

    /**
       \todo XXXX it should be possible to pass a point of interest
       (i.e. the current robot pose), such that we do not always
       compute the entire distance transform (i.e. when planning
       specific plans).
    */
    DistanceTransform * createDistanceTransform (double goalx, double goaly, int * opt_errcode);
    
  protected:
    sfl::GridFrame world_to_estar_;
    estar_t * estar_;
  };
  
}

#endif // CARGO_ANTS_UTIL_ESTAR_UTIL_HPP
