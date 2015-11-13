/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#include "GenericDrive.hpp"
#include "Object.hpp"
#include <cmath>

namespace npm2 {
  
  
  GenericDrive::
  GenericDrive (string const & name)
    : Actuator (name)
  {
  }
  
  
  void GenericDrive::
  setSpeed (double vtrans, double vrot)
  {
    vtrans_ = vtrans;
    vrot_ = vrot;
  }
  
  
  void GenericDrive::
  integrate (double dt)
  {
    if ( ! parent_) {
      return;
    }
    
    double const dth (vrot_ * dt);
    double dx, dy;
    if (fabs (dth) > 1.0e-4) {
      double const rr (vtrans_ / vrot_); // could check minimum turning radius... but then what?
      dx = rr * sin (dth);
      dy = rr * (1.0 - cos (dth));
    }
    else {
      dx = vtrans_ * dt;
      dy = 0.0;
    }
    
    parent_->motion_.Add (dx * parent_->motion_.Costheta() - dy * parent_->motion_.Sintheta(),
			  dx * parent_->motion_.Sintheta() + dy * parent_->motion_.Costheta(),
			  dth);
  }
  
}
