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


#ifndef SUNFLOWER_SPEEDOBJECTIVE_HPP
#define SUNFLOWER_SPEEDOBJECTIVE_HPP


#include <sfl/util/array2d.hpp>
#include <sfl/dwa/Objective.hpp>


namespace sfl {


  class RobotModel;
  
  
  class SpeedObjective
    : public Objective
  {
  public:
    SpeedObjective(const DynamicWindow & dynamic_window,
		   const RobotModel & robot_model);
    
    virtual void Initialize(std::ostream * progress_stream);
    
    virtual bool YieldsAdmissible() const { return false; }
    virtual bool Admissible(int qdlIndex, int qdrIndex) const { return true; }
    
    virtual void Calculate(double timestep, size_t qdlMin, size_t qdlMax,
			   size_t qdrMin, size_t qdrMax,
			   double carrot_lx, double carrot_ly,
			   boost::shared_ptr<const Scan> local_scan);
    
    void GoFast();
    void GoSlow();
    void GoForward();
    void GoBackward();
    void GoStrictFast();
    void GoStrictSlow();
    
    const double sdMax;
    
  protected:
    const RobotModel & m_robot_model;
    array2d<double> m_forward, m_backward, m_slow, m_strict_forward,
      m_strict_backward, m_strict_slow;
    array2d<double> * m_current;
    bool m_goForward;
  };
  
}

#endif // SUNFLOWER_SPEEDOBJECTIVE_HPP
