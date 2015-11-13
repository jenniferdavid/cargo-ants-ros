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


#ifndef SUNFLOWER_HEADINGOBJECTIVE_HPP
#define SUNFLOWER_HEADINGOBJECTIVE_HPP


#include <sfl/api/RobotModel.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/util/array2d.hpp>
#include <sfl/dwa/Objective.hpp>


namespace sfl {


  /**
     Objective to align motion with goal direction.

     The heading objective is what makes the dynamic window tend to chose
     motion commands that align the robot with the direction that leads to
     the goal. You can add an offset angle, which will make the heading
     objective optimal for the goal direction modified by that offset. The
     goal direction is expressed as a point in the robot's frame of
     reference, e.g. you can use subgoals to provide a globally consistant
     path if you like.
  */

  class HeadingObjective:
    public Objective
  {
  public:
    HeadingObjective(const DynamicWindow & dynamic_window,
		     const RobotModel & robot_model);
    
    virtual void Initialize(std::ostream * progress_stream) {}
    virtual bool YieldsAdmissible() const { return false; }
    virtual bool Admissible(int qdlIndex, int qdrIndex) const { return true; }
    
    virtual void Calculate(double timestep, size_t qdlMin, size_t qdlMax,
			   size_t qdrMin, size_t qdrMax,
			   double carrot_lx, double carrot_ly,
			   boost::shared_ptr<const Scan> local_scan);
    
    /** \pre (iqdl, iqdr) is admissible and was within the range of
	the previous call to Calculate(). */
    const Frame & PredictedStandstill(size_t iqdl, size_t iqdr) const;
    
    double local_goal_x, local_goal_y, angle_offset;
    
  protected:
    const RobotModel & m_robot_model;
    array2d<Frame> m_standstill_prediction;
  };

}

#endif // SUNFLOWER_HEADINGOBJECTIVE_HPP
