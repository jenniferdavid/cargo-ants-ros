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


#include "RobotDrawing.hpp"
#include <npm/RobotServer.hpp>
#include <npm/Object.hpp>
#include "wrap_gl.hpp"
#include <sfl/util/Line.hpp>


using namespace sfl;
using namespace boost;


namespace npm {
  
  
  RobotDrawing::
  RobotDrawing(const RobotServer * robot, color_s const & color)
    : Drawing(robot->GetName() + "_true_drawing",
	      "true (and noisy if available) outline of \"" + robot->GetName() + "\""),
      m_robot(robot),
      m_color(color)
  {
  }
  
  
  void RobotDrawing::
  Draw()
  {
    const Object & body(m_robot->GetBody());
    glColor3d(m_color.red, m_color.green, m_color.blue);
    glLineWidth(1);
    glBegin(GL_LINE_LOOP);
    for(size_t ii(0); ii < body.GetNlines(); ++ii){
      shared_ptr<const Line> line(body.GetGlobalLine(ii));
      glVertex2d(line->X0(), line->Y0());
      glVertex2d(line->X1(), line->Y1());
    }
    glEnd();
  }
  
}
