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


#ifndef SUNFLOWER_GOAL_HPP
#define SUNFLOWER_GOAL_HPP


#include <sfl/util/Frame.hpp>


namespace sfl {
  

  /**
     Geometrically defined goal (as opposed to node-ID or place
     name). A goal is defined by a pose (x, y, theta) and a precision
     (dr, dtheta) which effectively defines a goal-disk and a range of
     headings that are considered "at the goal".

     \note So-called "via goals" are potentially handled in a special
     way by the concrete MotionPlanner (e.g. the EXPO.02 MotionPlanner
     doesn't slow down inside a via goal's disk).
  */  
  class Goal
  {
  public:
    /**
       Default goal is at (0, 0, 0) with precision
       (Goal::DEFAULTGOALDR, Goal::DEFAULTGOALDTHETA).
    */
    Goal();

    /**
       Copy ctor.
    */
    Goal(const Goal & original);

    /**
       \note The viaGoal parameter might not be applicable to all
       implementations, so it defaults to <code>false</code>.
    */
    Goal(double x,
	 double y,
	 double theta,
	 double dr,
	 double dtheta,
	 bool viaGoal = false);
    
    void Set(const Goal & goal);

    void Set(double x,
	     double y,
	     double theta,
	     double dr,
	     double dtheta,
	     bool viaGoal = false);
    
    Goal & operator = (const Goal & rhs) {
      Set(rhs);
      return *this;
    }
    
    /**
       \return The goal's x-coordinate.
    */
    double X() const;
    
    /**
       \return The goal's y-coordinate.
    */
    double Y() const;
    
    /**
       \return The goal's theta-coordinate.
    */
    double Theta() const;
    
    /**
       \return The goal's radial tolerance.
    */
    double Dr() const;
    
    /**
       \return The square of Dr().
    */
    double Dr2() const;
    
    /**
       \return The goal's angular tolerance.
    */
    double Dtheta() const;
    
    /**
       \return Whether the goal is a "via goal".
    */
    bool IsVia() const;
    
    /**
       Checks whether the provided position lies inside the goal disk.
    */
    bool DistanceReached(const Frame & position) const;
    
    /**
       Checks whether the provided position is aligned with the goal
       heading (to within Dtheta()). The goForward parameter can be
       set to <code>false</code> if you want the robot to interpret
       the position as if it was moving backwards (this can be useful
       for tour-guiding applications).

       The difference in heading is returned in the reference
       parameter <code>dheading</code> so that client code does not
       have to recalculate it.
    */
    bool HeadingReached(const Frame & position,
			bool goForward,
			/** returned remaining heading error */
			double & dheading) const;
    
    /**
       Checks whether the goal has been reached. It relies on
       DistanceReached() and HeadingReached() for this purpose.
    */
    bool Reached(const Frame & position,
		 bool goForward) const;
    
    friend std::ostream & operator << (std::ostream & os, const Goal & goal);
    
    /**
       Default goal disk radius.
    */
    static const double DEFAULTGOALDR;
    
    /**
       Default alignment tolerance with the goal heading.
    */
    static const double DEFAULTGOALDTHETA;
    
    
  protected:
    friend class Frame;	// quick hack to compensate for historical design choices
    
    double _x, _y, _theta;
    double _dr, _dr2, _dtheta;
    bool _isVia;
  };
  
}

#endif // SUNFLOWER_GOAL_HPP
