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

#include "RayDistanceSensor.hpp"
#include <sfl/util/numeric.hpp>


namespace npm2 {
  
  
  RayDistanceSensor::
  RayDistanceSensor (string const & name)
    : Sensor (name),
      distance_ (1.0),
      max_distance_ (1.0)
  {
    reflectParameter ("max_distance", &max_distance_);
    reflectParameter ("distance", &distance_);
  }
  
  
  void RayDistanceSensor::
  sensorReset ()
  {
    distance_ = max_distance_;
  }
  
  
  void RayDistanceSensor::
  sensorUpdate (Body const & body)
  {
    // deciding which bodies to ignore could be smarter...
    //
    if ((&body == &body_) || (parent_ && (&body == &parent_->body_))) {
      return;
    }
    
    for (size_t il(0); il < body.getLines().size(); ++il) {
      Line const & ll (body.getLines()[il]);
      double const dd (LineRayIntersect (ll.X0(), ll.Y0(), ll.X1(), ll.Y1(),
					 global_.X(), global_.Y(),
					 global_.Costheta(), global_.Sintheta()));
      if ((dd > 0) && (dd < distance_)) {
	distance_ = dd;
      }
    }
  }
  
}
