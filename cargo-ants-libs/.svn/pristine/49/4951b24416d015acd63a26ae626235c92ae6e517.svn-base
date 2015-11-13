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


#include "SharpDrawing.hpp"
#include <npm/Sharp.hpp>
#include "wrap_gl.hpp"
#include <sfl/util/Ray.hpp>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  SharpDrawing::
  SharpDrawing(const string & name, const Sharp & sharp)
    : Drawing(name,
	      "1D sharp distance data in global reference frame"),
      m_sharp(sharp)
  {
  }



  void SharpDrawing::
  Draw()
  {
    shared_ptr<const Ray> ray(m_sharp.GetRay());
    if( ! ray)
      return;
    glColor3d(1, 0.5, 0.5);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex2d(ray->X(), ray->Y());
    glVertex2d(ray->X() + ray->Dx(), ray->Y() + ray->Dy());
    glEnd();
  }

}
