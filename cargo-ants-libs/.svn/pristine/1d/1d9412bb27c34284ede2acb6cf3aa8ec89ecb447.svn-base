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

#ifndef EXPO_ROBOX_HPP
#define EXPO_ROBOX_HPP

#include <boost/shared_ptr.hpp>
#include <stdexcept>

namespace sfl {
  class DynamicWindow;
  class DistanceObjective;
  class SpeedObjective;
  class HeadingObjective;
  class RobotModel;
  class Hull;
  class BubbleBand;
  class Odometry;
  class Scanner;
  class Multiscanner;
  class Goal;
  class Pose;
  class LocalizationInterface;
  class DiffDriveChannel;
}


namespace expo {

  struct expo_parameters;
  class MotionPlanner;
  class MotionController;
  
  
  class Robox
  {
  public:
    Robox(expo_parameters const & params,
	  /** If you do not have a special hull, then simply use
	      expo::Robox::CreateDefaultHull() here. */
	  boost::shared_ptr<sfl::Hull> hull,
	  boost::shared_ptr<sfl::LocalizationInterface> localization,
	  boost::shared_ptr<sfl::DiffDriveChannel> drive,
	  boost::shared_ptr<sfl::Multiscanner> mscan);
    
    void SetGoal(double timestep, sfl::Goal const & goal);
    
    sfl::Goal const & GetGoal() const;
    
    /** Calls UpdateMultiscanner(), UpdateMotionPlanner(),
	UpdateMotionController(), and UpdateOdometry(). It throws an
	exception if anything goes wrong. */
    void Update(double timestep) throw(std::runtime_error);
    
    bool UpdateMultiscanner(std::ostream * erros);
    
    bool UpdateMotionPlanner(double timestep);
    
    int UpdateMotionController(double timestep, std::ostream * err_os);
    
    static boost::shared_ptr<sfl::Hull> CreateDefaultHull();
    
    
    boost::shared_ptr<sfl::Hull> hull;
    boost::shared_ptr<sfl::RobotModel> robotModel;
    boost::shared_ptr<MotionController> motionController;
    boost::shared_ptr<sfl::DynamicWindow> dynamicWindow;
    boost::shared_ptr<sfl::DistanceObjective> distanceObjective;
    boost::shared_ptr<sfl::HeadingObjective> headingObjective;
    boost::shared_ptr<sfl::SpeedObjective> speedObjective;
    boost::shared_ptr<sfl::Odometry> odometry;
    boost::shared_ptr<sfl::BubbleBand> bubbleBand;
    boost::shared_ptr<sfl::Multiscanner> mscan;
    boost::shared_ptr<MotionPlanner> motionPlanner;
  };

}

#endif // EXPO_ROBOX_HPP
