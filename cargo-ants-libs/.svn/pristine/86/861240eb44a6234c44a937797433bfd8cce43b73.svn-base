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

#ifndef NPM2_RAY_DISTANCE_SENSOR_HPP
#define NPM2_RAY_DISTANCE_SENSOR_HPP

#include <npm2/Sensor.hpp>


namespace npm2 {
  
  
  class RayDistanceSensor
    : public Sensor
  {
  public:
    explicit RayDistanceSensor (string const & name);
    
    virtual void sensorReset ();
    
    virtual void sensorUpdate (Body const & body);
    
    double distance_;
    double max_distance_;
  };
  
}

#endif // NPM2_RAY_DISTANCE_SENSOR_HPP
