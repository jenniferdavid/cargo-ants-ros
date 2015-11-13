/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#include "MapperUpdateDrawing.hpp"
#include "wrap_gl.hpp"
#include <npm/pdebug.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <cmath>
#include <err.h>


using namespace sfl;


namespace npm {
  
  
  MapperUpdateDrawing::
  MapperUpdateDrawing(const std::string & name,
											what_t _what,
											boost::shared_ptr<const sfl::Mapper2d> mapper)
    : Drawing(name,
							"most recent changes of a sweeping-update sfl::Mapper2d"),
			what(_what),
      m_mapper(mapper)
  {
  }
  

  void MapperUpdateDrawing::
  Draw()
  {
    Mapper2d::index_buffer_t const * buf;
		double offset;
		switch (what) {
		case FREESPACE:
			buf = &m_mapper->GetFreespaceBuffer();
			offset = 0.1;
			glColor3d(0, 1, 0.5);
			break;
		case OBSTACLE:
			buf = &m_mapper->GetObstacleBuffer();
			offset = 0.15;
			glColor3d(1, 0, 0.5);
			break;
		case CHECK:
			buf = &m_mapper->GetSwipeCheckBuffer();
			offset = 0.2;
			glColor3d(0.5, 0, 1);
			break;
		case HOLE:
			buf = &m_mapper->GetSwipeHoleBuffer();
			offset = 0.25;
			glColor3d(0.5, 1, 0);
			break;
		case REPAIR:
			buf = &m_mapper->GetSwipeRepairBuffer();
			offset = 0.3;
			glColor3d(0, 0.5, 1);
			break;
		default:
			warnx("npm::MapperUpdateDrawing::Draw(): invalid what %d", what);
			return;
		}
		
    if (buf->empty())
      return;
    
    const GridFrame & gframe(m_mapper->GetGridFrame());
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(gframe.X(), gframe.Y(), 0);
    glRotated(180 * gframe.Theta() / M_PI, 0, 0, 1);
    glScaled(gframe.Delta(), gframe.Delta(), 1);
		glLineWidth(1);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		
    for(Mapper2d::index_buffer_t::const_iterator ibuf(buf->begin()); ibuf != buf->end(); ++ibuf){
      PVDEBUG("buf %zd %zd\n", ibuf->v0, ibuf->v1);
      glRectd(ibuf->v0 - offset, ibuf->v1 - offset,
							ibuf->v0 + offset, ibuf->v1 + offset);
    }
    
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();    
  }

}
