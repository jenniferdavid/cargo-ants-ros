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

#ifndef NPM2_REVOLUTE_SERVO_HPP
#define NPM2_REVOLUTE_SERVO_HPP

#include <npm2/Actuator.hpp>


namespace npm2 {
  
  
  class RevoluteServo
    : public Actuator
  {
  public:
    explicit RevoluteServo (string const & name);
    
    void setAngle (double aa);
    
    virtual void integrate (double dt);
    
  protected:
    double angle_;
  };
  
}

#endif // NPM2_REVOLUTE_SERVO_HPP
