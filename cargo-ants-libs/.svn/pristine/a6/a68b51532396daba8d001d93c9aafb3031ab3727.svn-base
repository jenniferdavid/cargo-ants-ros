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


#include "Sharp.hpp"
#include "gfx/SharpDrawing.hpp"
#include "RobotServer.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/util/Ray.hpp>
#include <sfl/util/Frame.hpp>


using namespace sfl;


namespace npm {


  Sharp::
  Sharp(const RobotServer * robot, const sfl::Frame & mount,
	double _rmax, int channel)
    : Sensor(robot), rmax(_rmax), m_mount(new Frame(mount)),
      m_drawing(new SharpDrawing(robot->GetName() + "_sharp"
				 + to_string(channel), * this))
  {
  }
  
  
  void Sharp::
  InitUpdate()
  {
    Frame trans(*m_mount);
    owner->GetPose().To(trans);
    m_ray.reset(new Ray(trans));
    m_rho = rmax;
  }
  
  
  void Sharp::
  StepUpdate(const Line & line)
  {
    if( ! m_ray)
      return;
    const double dd(m_ray->Intersect(line));
    if((dd > 0) && (dd < m_rho))
      m_rho = dd;
  }
  
}
