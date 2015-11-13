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


#ifndef SUNFLOWER_MOTIONPLANNER_HPP
#define SUNFLOWER_MOTIONPLANNER_HPP


namespace sfl {
  
  
  class Goal;
  
  
  /**
     The MotionPlanner manages path planning and obstacle
     avoidance. It is the place where clients set the goal and can
     find out if the robot has reached it.

     \note This is a <em>pure abstract</em>
     class. expo::MotionPlanner, an actual implementation, can be
     found in the <code>expo/</code> subdirectory.
  */
  class MotionPlanner
  {
  public:
    /** empty default implementation */
    virtual ~MotionPlanner() {}
  
    /** define the robot's goal */
    virtual void SetGoal(double timestep, const Goal & goal) = 0;

    /** \return The robot's current goal. */
    virtual const Goal & GetGoal() const = 0;

    /** Query if the robot has reached its goal. */
    virtual bool GoalReached() const = 0;
  };
  
}

#endif // SUNFLOWER_MOTIONPLANNER_HPP
