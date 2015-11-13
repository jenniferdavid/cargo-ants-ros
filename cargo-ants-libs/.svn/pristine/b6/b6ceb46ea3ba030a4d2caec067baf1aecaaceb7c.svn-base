/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef NPM_ROBOTCLIENT_HPP
#define NPM_ROBOTCLIENT_HPP

#include <sfl/api/Goal.hpp>
#include <fpplib/configurable.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {
  class Line;
  class Polygon;
  class Frame;
  class Scanner;
}


namespace npm {
  
  class Lidar;
  class Sharp;
  class Drawing;
  class Camera;
  class DiffDrive;
  class HoloDrive;
  class BicycleDrive;
  class RobotServer;
  

  /**
     Just a quick refactoring hack.
     
     \todo XXXX to do: this should be implemented by making sfl::Goal
     and/or sfl::GoalManager configurable via fpplib.
   */
  struct qhgoal_s {
    double x, y, theta, dr, dtheta;
  };

  /** \todo XXXX to do: just a quick hack... should extend sfl::Frame instead */
  struct qhpose_s {
    double x, y, theta;
  };
  
  
  struct color_s {
    color_s ()
      : red (1.0), green (1.0), blue (1.0)
    {}
    
    color_s (double r, double g, double b)
      : red (r), green (g), blue (b)
    {}
    
    color_s (color_s const & rhs)
      : red (rhs.red), green (rhs.green), blue (rhs.blue)
    {}
    
    double red, green, blue;
  };
  
  
  /**
     Base class for implementing robots.
  */
  class RobotClient
    : public fpplib::Configurable
  {
  public:
    typedef fpplib::Registry <RobotClient, false> registry_t;
    static registry_t registry;
    
    RobotClient(std::string const &name);
    
    virtual bool Initialize(RobotServer &server);
    
    /** Entry point for simulating the robot. It should calculate the
	next action based on sensor readings (which are updated by the
	simulator) and should end by assigning new motor commands to the
	actuators.
	
	\return true if the robot is in a runnable state. If you
	return false, the simulator will switch to step-by-step
	mode. It is up to subclasses to print appropriate error
	message or try to recover.
    */
    virtual bool PrepareAction(double timestep) = 0;
    
    /** Hook for initially placing the robot, before simulation
	starts. Subclasses can provide an empty implementation if they
	don't track the pose. */
    virtual void InitPose(sfl::Frame const &pose) = 0;
    
    /** Hook for telling the robot where it is, after simulation has
	started. This could be used to simulate e.g. GPS-based
	updates. Subclasses can provide an empty implementation if
	they do not track the pose. */
    virtual void SetPose(sfl::Frame const &pose) = 0;
    
    /** Hook for knowing where the robot thinks it is. Subclasses can
	implement this simpluy by returning false, which signifies
	that the robot has no estimate of its position. */
    virtual bool GetPose(sfl::Frame &pose) const = 0;
    
    /** Hook to set the robot's goal. Subclasses can use an empty
	implementation if they do not handle explicit goals. */
    virtual void SetGoal(double timestep, const sfl::Goal & goal) = 0;
    
    /** Hook to query the robot's current goal. Subclasses can just
	return false here, which signifies that the robot has no
	(explicitly maintained) goal. */
    virtual bool GetGoal(sfl::Goal &goal) const = 0;
    
    /** Hook for knowing when the robot thinks it has reached its
	goal. It is okay to simply always return false here, in case
	the subclass does not use explicit goals. */
    virtual bool GoalReached() const = 0;
    
    color_s const & GetColor() const { return m_color; }
    
  private:
    friend class RobotServer;
    friend class Simulator;	// quick hack due to initial pose hack
    
    bool m_enable_trajectory;
    
    bool m_noisy_scanners;
    double m_scanner_noise_min_factor; // factors <0 are ignored
    double m_scanner_noise_max_factor;
    double m_scanner_noise_min_offset; // if min>max then offsets are ignored
    double m_scanner_noise_max_offset;
    
    double m_camera_zoom;
    
    std::vector<sfl::Goal> m_goals;
    bool AppendGoal(qhgoal_s const &goal);
    
    qhpose_s m_initial_pose;
    
    color_s m_color;
  };
  
}

namespace std {
  
  ostream & operator << (ostream &os, npm::qhgoal_s const &rhs);
  ostream & operator << (ostream &os, npm::qhpose_s const &rhs);
  ostream & operator << (ostream &os, npm::color_s const &rhs);

}

#endif // NPM_ROBOTCLIENT_HPP
