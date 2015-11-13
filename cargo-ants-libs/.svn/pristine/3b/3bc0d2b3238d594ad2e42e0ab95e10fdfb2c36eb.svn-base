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


#ifndef NPM_SHARP_HPP
#define NPM_SHARP_HPP


#include <npm/Sensor.hpp>


namespace sfl {
  class Ray;
  class Frame;
}


namespace npm {

  class SharpDrawing;
  
  /**
     Simulates a single ray distance sensor.
  */
  class Sharp
    : public Sensor
  {
  private:
    friend class RobotServer;
    
    /** The constructor is private such that only the friend Robot can
	create sharps. This is accomplished through
	Robot::DefineSharp(). */
    Sharp(const RobotServer * robot, const sfl::Frame & mount,
	  double rmax, int channel);
    
  public:
    virtual void InitUpdate();
    virtual void StepUpdate(const sfl::Line & line);
    
    double Get() const { return m_rho; }
    boost::shared_ptr<const sfl::Ray> GetRay() const { return m_ray; }
    
    const double rmax;
    
  protected:
    boost::shared_ptr<const sfl::Frame> m_mount;
    boost::shared_ptr<sfl::Ray> m_ray;
    boost::shared_ptr<SharpDrawing> m_drawing;
    double m_rho;
  };

}

#endif // NPM_SHARP_HPP
