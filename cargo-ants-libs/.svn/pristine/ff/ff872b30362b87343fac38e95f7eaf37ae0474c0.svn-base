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


#include "DiffDriveDrawing.hpp"
#include "wrap_gl.hpp"
#include <sfl/util/Frame.hpp>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  DiffDriveDrawing::
  DiffDriveDrawing(const string & name,
		   shared_ptr<const DiffDrive> drive)
    : Drawing(name,
	      "schematic of DiffDrive at current true robot pose"),
      m_drive(drive),
      m_halfwheelbase(drive->wheelbase / 2)
  {
  }


  void DiffDriveDrawing::
  Draw()
  {
    glColor3d(1, 1, 1);
    glLineWidth(1);
    glBegin(GL_LINES);

    shared_ptr<const sfl::Frame> trans(m_drive->PoseCache());

    double x(m_halfwheelbase);
    double y(0);
    trans->To(x, y);
    glVertex2d(trans->X(), trans->Y());
    glVertex2d(x, y);

    x = 0;
    y = m_halfwheelbase;
    trans->To(x, y);
    glVertex2d(trans->X(), trans->Y());
    glVertex2d(x, y);

    x = 0;
    y = - m_halfwheelbase;
    trans->To(x, y);
    glVertex2d(trans->X(), trans->Y());
    glVertex2d(x, y);
  
    glEnd();
  }

}
