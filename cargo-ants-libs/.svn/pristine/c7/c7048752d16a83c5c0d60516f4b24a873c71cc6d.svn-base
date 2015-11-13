/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#ifndef SUNFLOWER_MOTIONCONTROLLER_HPP
#define SUNFLOWER_MOTIONCONTROLLER_HPP


#include <sfl/api/DiffDriveChannel.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {
  
  
  class RobotModel;
  
  
  /**
     Encapsulates the motion controller. Used to control the robot's
     movement.
     
     \todo hard-coded for differential-drive robots...
  */
  class MotionController
  {
  public:
    MotionController(boost::shared_ptr<const RobotModel> robotModel,
		     boost::shared_ptr<DiffDriveChannel> drive);
    
    /**
       Template method for determining the next motion command. The
       speeds are set through ProposeSpeed() or
       ProposeActuators(). Gets current speeds from DiffDriveChannel, applies
       kinodynamic limits to proposed speeds, and passes them to DiffDriveChannel.
       
       \return 0 on success, -1 if DiffDriveChannel::GetSpeed() failed, -2 if
       DiffDriveChannel::SetSpeed() failed.
    */
    int Update(/** (estimated or fixed) delay until next invocation */
	       double timestep,
	       /** if non-zero, debug messages are written to dbgos */
	       std::ostream * dbgos = 0);
    
    /** Set the (global) speed to be applied during the next Update(). */
    void ProposeSpeed(double sd, double thetad);
    
    /** Set the actuator speeds for next Update(). This is more direct
	than ProposeSpeed(). */
    void ProposeActuators(double qdLeft, double qdRight);
    
    /** Get the current (global) speed, as of the previous Update(). */
    void GetCurrentGlob(double & sd, double & thetad) const;
    
    /** Get the wanted (global) speed, as of the previous Update(). */
    void GetWantedGlob(double & sd, double & thetad) const;
    
    /** Get the proposed speed, as of the previous ProposeActuators()
	or ProposeSpeed(). */
    void GetProposedGlob(double & sd, double & thetad) const;
    
    /** Get the current actuator speeds, as of the previous Update(). */
    void GetCurrentAct(double & qdLeft, double & qdRight) const;
    
    /** Get the wanted actuator speeds, as of the previous Update(). */
    void GetWantedAct(double & qdLeft, double & qdRight) const;
    
    /** Get the proposed actuator speeds, as of the previous
	ProposeActuators() or ProposeSpeed(). */
    void GetProposedAct(double & qdLeft, double & qdRight) const;
    
    const double qdMax;
    const double qddMax;
    const double sdMax;
    const double thetadMax;
    
  protected:
    boost::shared_ptr<const RobotModel> m_robotModel;
    boost::shared_ptr<DiffDriveChannel> m_drive;
    double m_proposedQdl, m_proposedQdr;
    double m_currentQdl, m_currentQdr;
    double m_wantedQdl, m_wantedQdr;
  };
  
}

#endif // SUNFLOWER_MOTIONCONTROLLER_HPP
