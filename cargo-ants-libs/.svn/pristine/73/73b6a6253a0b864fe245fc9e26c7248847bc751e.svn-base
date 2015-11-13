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


#include "WorldDrawing.hpp"
#include <npm/World.hpp>
#include <npm/Object.hpp>
#include "wrap_gl.hpp"
#include <sfl/util/Line.hpp>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  WorldDrawing::
  WorldDrawing(const string & name, const World & world)
    : Drawing(name,
	      "all lines in the world, including objects and robots"),
      m_world(world)
  {
  }
  
  
  void WorldDrawing::
  Draw()
  {
    const World::object_t & object(m_world.GetObjects());
    if(object.empty())
      return;
    glLineWidth(1);
    glBegin(GL_LINES);
    glColor3d(1, 1, 1);
    for(size_t io(0); io < object.size(); ++io)
      for(size_t il(0); il < object[io]->GetNlines(); ++il){
	shared_ptr<const Line> line(object[io]->GetGlobalLine(il));
	glVertex2d(line->X0(), line->Y0());
	glVertex2d(line->X1(), line->Y1());
      }
    glEnd();
  }

}
