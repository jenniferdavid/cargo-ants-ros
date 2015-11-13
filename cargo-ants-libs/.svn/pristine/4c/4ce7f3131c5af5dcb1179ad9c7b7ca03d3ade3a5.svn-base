/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#include "BicycleDrive.hpp"
#include <sfl/util/Frame.hpp>
#include <sfl/util/numeric.hpp>


using namespace sfl;
using namespace boost;


namespace npm {
  
  
  BicycleDrive::
  BicycleDrive(double _wheelbase,
	       double _wheelradius, double _axlewidth)
    : wheelbase(_wheelbase), wheelradius(_wheelradius),
      axlewidth(_axlewidth),
      v_trans(0.0), steer(0.0)
  {
  }
  
  
  shared_ptr<Frame> BicycleDrive::
  ComputeNextPose(const Frame & current, double timestep) const
  {
    if(absval(v_trans) < epsilon) // otherwise we get NaN results...
      return shared_ptr<Frame>(new Frame(current));
    
    double dx, dy, dtheta;
    if(absval(steer) < epsilon * 2 * M_PI){
      // straight line
      dx = v_trans * timestep;
      dy = 0;
      dtheta = 0;
    }
    else{
      // arc of circle
      // tan(steer) is OK as of check for M_PI/2 above
      const double R(wheelbase / tan(steer));
      dtheta = v_trans * timestep * tan(steer) / wheelbase;
      dx = R * sin(dtheta);
      dy = R * (1 - cos(dtheta));
    }
    current.RotateTo(dx, dy);
    
    shared_ptr<Frame> result(new Frame(current));
    result->Add(dx, dy, dtheta);
    return result;
  }
  
}
