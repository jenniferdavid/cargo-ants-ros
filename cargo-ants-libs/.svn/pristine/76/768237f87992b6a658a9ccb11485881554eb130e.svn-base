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


#ifndef EXPO_MOTIONCONTROLLER_HPP
#define EXPO_MOTIONCONTROLLER_HPP


#include <sfl/api/MotionController.hpp>


namespace expo {
  
  
  class MotionController:
    public sfl::MotionController
  {
  public:
    MotionController(boost::shared_ptr<const sfl::RobotModel> robotModel,
		     boost::shared_ptr<sfl::DiffDriveChannel> drive);
    
    /** \note This is a bit of a hack that never really performed well. */
    bool AlmostStraight() const;
    bool Stoppable(double timestep) const;
    bool Moving() const;
  };

}

#endif // EXPO_MOTIONCONTROLLER_HPP
