/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#ifndef NPM_DIFFDRIVE_HPP
#define NPM_DIFFDRIVE_HPP


#include <npm/Drive.hpp>


namespace sfl {
  class DiffDriveChannel;
}


namespace npm {
  
  /**
     Differential drive actuator.
  */
  class DiffDrive
    : public Drive
  {
  public:
    DiffDrive(double wheelbase, double wheelradius);
    
    boost::shared_ptr<sfl::DiffDriveChannel> CreateChannel();
    
    /** distance between wheel contact points [m] */
    const double wheelbase;
    
    /** radius of drive wheels [m] */
    const double wheelradius;
    
    double qdl, qdr;
    
  protected:
    virtual boost::shared_ptr<sfl::Frame>
    ComputeNextPose(const sfl::Frame & current, double timestep) const;
  };
  
}

#endif // NPM_DIFFDRIVE_HPP
