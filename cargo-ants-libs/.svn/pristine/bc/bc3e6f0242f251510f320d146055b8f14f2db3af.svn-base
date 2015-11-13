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

#ifndef NPM2_DIFFERENTIAL_TRAILER_DRIVE_HPP
#define NPM2_DIFFERENTIAL_TRAILER_DRIVE_HPP

#include <npm2/DifferentialDrive.hpp>


namespace npm2 {
  
  
  /**
     See Lamiraux, Sekhavat, and Laumond. Motion Planning and Control
     for Hilare Pulling a Trailer. IEEE Transaction on Robotics and
     Automation 15(4), August 1999.
  */
  class DifferentialTrailerDrive
    : public DifferentialDrive
  {
  public:
    explicit DifferentialTrailerDrive (string const & name);
    
    virtual void integrate (double dt);
    
    double getTrailerAngle () const { return trailer_angle_; }
    
    double hitch_offset_;	// l_r (usually positive, which means means "behind" axle)
    double trailer_arm_;	// l_t (usually positive)
    Object * trailer_;
    
  protected:
    double trailer_angle_;
  };
  
}

#endif // NPM2_DIFFERENTIAL_TRAILER_DRIVE_HPP
