/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
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

#ifndef SFL_GOALMANAGER_HPP
#define SFL_GOALMANAGER_HPP

#include <boost/shared_ptr.hpp>
#include <string>
#include <iosfwd>
#include <vector>

namespace sfl {

  class Goal;
  class Frame;

  class GoalManager {
  public:
    /** If and how to cycle through the registered goals. The default
				is to loop. */
    typedef enum { LOOP, NONE /*, RAND */ } repeat_t;
    
    GoalManager();
    
    /** \return true if the config file was free of errors. If os is
				non-null, error messages are written into it.
				
				\note If false is returned, the state of the GoalManager
				instance is not well defined (you should throw it away).
    */
    bool ParseConfig(const std::string & filename, std::ostream * os);
		
    
    /** Like the other ParseConfig(), but takes an already opened
				stream as input. */
    bool ParseConfig(std::istream & is, std::ostream * os);
		
		/* parsing a simple goal file */
		bool ParseConfigSimple(const std::string & filename, std::ostream * os,
													 const std::string & mode, double goal_theta,
													 double goal_radius, double goal_theta_diff);
		
		bool ParseConfigSimple(std::istream & is, std::ostream * os, const std::string & mode,
													 double goal_theta, double goal_radius, double goal_theta_diff);
    
    /** \return 0 iff no goals are registered or the last goal has
				been reached, a valid pointer otherwise. */
    boost::shared_ptr<Goal> GetCurrentGoal() const;
    
    /** Simply appends to the list. */
    void AddGoal(double x, double y, double theta, double dr, double dtheta);
    
    /** Used to cycle through the goals registered with AddGoal(). */
    void NextGoal();
    
    /** \return true iff there is a current goal and the given pose
				reaches that goal. */
    bool GoalReached(double x, double y, double theta, bool go_forward) const;
    
    /** Same as the other GoalReached(), but convenient if you already
				have an instance of sfl::Frame or sfl::Pose. */
    bool GoalReached(const Frame & pose, bool go_forward) const;
		
  private:
    /** \note We keep shared ptrs because GetCurrentGoal() returns a
				pointer, which might be invalidated by AddGoal() if we kept
				instances in a std::vector. So GetCurrentGoal() has to return
				a shared ptr as well, to avoid problems if someone wants to
				store that for later use (returning a raw pointer would cause
				double-deletes). */
    typedef std::vector<boost::shared_ptr<Goal> > goal_t;
    
    repeat_t m_repeat;
    goal_t m_goal;
    size_t m_current_goal;
  };

}

#endif // SFL_GOALMANAGER_HPP
