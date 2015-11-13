/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#include "ObjectDrawing.hpp"
#include "View.hpp"
#include "gl.hpp"
#include "RayDistanceSensor.hpp"


namespace npm2 {
  

  ObjectDrawing::
  ObjectDrawing (string const & name)
    : Drawing (name, "default body and sensor visualization of a tree of objects"),
      object_ (0)
  {
    reflectSlot ("object", &object_);
  }
  
  
  static void recurse_draw (Object const * obj)
  {
    glLineWidth (2);
    glColor3d (1.0, 1.0, 1.0);
    glBegin (GL_LINES);
    Body::lines_t const & lines (obj->body_.getLines());
    for (size_t il(0); il < lines.size(); ++il) {
      glVertex2d (lines[il].X0(), lines[il].Y0());
      glVertex2d (lines[il].X1(), lines[il].Y1());
    }
    glEnd ();
    
    RayDistanceSensor const * rds (dynamic_cast <RayDistanceSensor const *> (obj));
    if (rds) {
      glLineWidth (1);
      glColor3d (1.0, 0.0, 0.0);
      glBegin (GL_LINES);
      glVertex2d (rds->getGlobal().X(),
		  rds->getGlobal().Y());
      glVertex2d (rds->getGlobal().X() + rds->distance_ * rds->getGlobal().Costheta(),
		  rds->getGlobal().Y() + rds->distance_ * rds->getGlobal().Sintheta());
      glEnd ();
      glPointSize (3);
      glColor3d (1.0, 0.5, 0.0);
      glVertex2d (rds->getGlobal().X() + rds->distance_ * rds->getGlobal().Costheta(),
		  rds->getGlobal().Y() + rds->distance_ * rds->getGlobal().Sintheta());
      glEnd();
    }
    
    for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
      recurse_draw (*ic);
    }
  }
  
  
  void ObjectDrawing::
  draw ()
  {
    if (object_) {
      recurse_draw (object_);
    }
  }
  
}
