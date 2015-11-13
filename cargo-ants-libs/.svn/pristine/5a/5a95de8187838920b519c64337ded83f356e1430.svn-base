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


#include "OdometryDrawing.hpp"
#include "wrap_gl.hpp"
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Pose.hpp>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  OdometryDrawing::
  OdometryDrawing(const string & name,
		  const Odometry & odometry,
		  double size)
    : Drawing(name,
	      "the current pose estimation of a robot using sfl::Odometry"),
      m_odometry(odometry),
      m_size(size)
  {
  }


  void OdometryDrawing::
  Draw()
  {
    glColor3d(0, 1, 0);

    //   glPointSize(5);
    //   glBegin(GL_POINTS);
    //   glVertex2d(m_odometry.Get().X(), m_odometry.Get().Y());
    //   glEnd();

    glLineWidth(1);
    glBegin(GL_LINES);

    double x(m_size);
    double y(0);
    shared_ptr<const Pose> pose(m_odometry.Get());
    pose->To(x, y);
    glVertex2d(pose->X(), pose->Y());
    glVertex2d(x, y);

    x = 0;
    y = m_size;
    pose->To(x, y);
    glVertex2d(pose->X(), pose->Y());
    glVertex2d(x, y);

    x = 0;
    y = - m_size;
    pose->To(x, y);
    glVertex2d(pose->X(), pose->Y());
    glVertex2d(x, y);
  
    glEnd();
  }

}
