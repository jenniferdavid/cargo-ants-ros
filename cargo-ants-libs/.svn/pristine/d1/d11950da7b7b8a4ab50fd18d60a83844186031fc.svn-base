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

#ifndef NPM_BICYCLE_DRIVE_HPP
#define NPM_BICYCLE_DRIVE_HPP

#include <npm/Drive.hpp>

namespace npm {
  
  /**
     Bicycle drive actuator.
 
     \code
      /       /
     /---+---/
    /    |  /
         |
         |wheelbase
         |
     |   |   |
     |---+---|
     |       |
     axlewidth
     \endcode
  */
  
  class BicycleDrive
    : public Drive
  {
  public:
    BicycleDrive(double wheelbase,
		 double wheelradius,
		 double axlewidth);
    
    const double wheelbase;
    const double wheelradius;
    const double axlewidth;
    
    double v_trans;
    double steer;
    
  protected:
    virtual boost::shared_ptr<sfl::Frame>
    ComputeNextPose(const sfl::Frame & current, double timestep) const;
  };
  
}

#endif // NPM_BYCICLE_DRIVE_HPP
