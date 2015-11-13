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

#ifndef NPM2_SENSOR_HPP
#define NPM2_SENSOR_HPP

#include <npm2/Object.hpp>


namespace npm2 {
  
  
  class Sensor
    : public Object
  {
  public:
    explicit Sensor (string const & name);
    
    virtual void sensorReset () = 0;
    virtual void sensorUpdate (Body const & body) = 0;
  };
  
}

#endif // NPM2_SENSOR_HPP
