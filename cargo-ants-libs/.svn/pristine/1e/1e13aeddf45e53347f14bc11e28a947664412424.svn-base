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


#include "ArcDrawing.hpp"
#include "Smart.hpp"
#include "../common/wrap_gl.hpp"
#include "../common/pdebug.hpp"
#include "../common/Manager.hpp"
#include <asl/ArcControl.hpp>


using namespace asl;
using namespace sfl;
using namespace npm;
using namespace boost;


ArcDrawing::
ArcDrawing(const std::string & name,
					 Smart * smart,
					 bool enabled,
					 bool recomp_status)
  : Drawing(name,
						"arcs of TADPF cntrl (cached or recomputed)",
						Instance<UniqueManager<Drawing> >()),
    m_smart(smart),
    m_enabled(enabled),
    m_recomp_status(recomp_status)
{
}


void ArcDrawing::
Draw()
{
  if( ! m_enabled){
    PVDEBUG("disabled\n");
    return;
  }
  const ArcControl * arc_control(m_smart->GetArcControl());
  if( ! arc_control){
    PVDEBUG("no arc control\n");
    return;
  }
  const std::vector<Arc> & arc_set(arc_control->GetArcSet());
  if(arc_set.empty()){
    PVDEBUG("empty arc set\n");
    return;
  }
  
  shared_ptr<const NavFuncQuery> query;
  if(m_recomp_status){
    query = m_smart->GetQuery();
    if( ! query){
      PDEBUG("m_recomp_status but no query object!\n");
      return;
    }
  }
  
  double px, py, ptheta;
  m_smart->GetPose(px, py, ptheta);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(px, py, 0);
  glRotated(180 * ptheta / M_PI, 0, 0, 1);
  const Frame pose(px, py, ptheta);
  
  PVDEBUG("arc_set.size(): %lu\n", arc_set.size());
  for(size_t ii(0); ii < arc_set.size(); ++ii){
    const Arc & arc(arc_set[ii]);
    const int numpts(arc.GetNumPts());
    if(numpts < 1)
      continue;
		const int last_good(arc.GetLastGoodState());
		if( ! m_recomp_status){
			if( ! arc.arc_valid)
				continue;
		}
    glPointSize(1);
    glBegin(GL_POINTS);
    for(int jj(1); jj < numpts; ++jj){
      const pt2d pt(arc.GetPoint(jj));
      NavFuncQuery::status_t status;
      if( ! m_recomp_status){
				if(jj > last_good)
					break;
				status = arc.GetNavFuncStat();
			}
			else{
				const pt2d pt0(arc.GetPoint(jj-1));
				double gx0(pt0.v0);
				double gy0(pt0.v1);
				double gx1(pt.v0);
				double gy1(pt.v1);
				pose.To(gx0, gy0);
				pose.To(gx1, gy1);
				double delta;
				////	status = query->GetValue(gx, gy, value);
				status = query->ComputeDeltaCost(gx0, gy0, gx1, gy1, delta);
      }
      switch(status){
      case NavFuncQuery::SUCCESS:
				glColor3d(0, 1, 0);	// green
				break;
      case NavFuncQuery::OUT_OF_BOUNDS:
				glColor3d(0.5, 0, 0);	// dark red
				break;
      case NavFuncQuery::OBSTACLE:
				glColor3d(1, 0.2, 0.2);	// bright red
				break;
      case NavFuncQuery::UNPROPAGATED:
				glColor3d(1, 0.2, 1);	// bright magenta
				break;
      case NavFuncQuery::INVALID:
				glColor3d(0, 1, 1);	// cyan
				break;
      default:
				PVDEBUG("unhandled NavFuncQuery::status_t %d\n", status);
				glColor3d(0.5, 0.5, 0.5); // grey
      }
      glVertex2d(pt.v0, pt.v1);
    }
    glEnd();
  }
  
	if( ! m_recomp_status){
		for(size_t ii(0); ii < arc_set.size(); ++ii){
			const Arc & arc(arc_set[ii]);
			const int last_good(arc.GetLastGoodState());
			if( ! arc.arc_valid)
				continue;
			if(last_good < 1)
				continue;
			glPointSize(4);
			glBegin(GL_POINTS);
      const pt2d pt(arc.GetPoint(last_good));
      NavFuncQuery::status_t status;
			status = arc.GetNavFuncStat();
      switch(status){
      case NavFuncQuery::SUCCESS:
				glColor3d(0, 1, 0);	// green
				break;
      case NavFuncQuery::OUT_OF_BOUNDS:
				glColor3d(0.5, 0, 0);	// dark red
				break;
      case NavFuncQuery::OBSTACLE:
				glColor3d(1, 0.2, 0.2);	// bright red
				break;
      case NavFuncQuery::UNPROPAGATED:
				glColor3d(1, 0.2, 1);	// bright magenta
				break;
      case NavFuncQuery::INVALID:
				glColor3d(0, 1, 1);	// cyan
				break;
      default:
				PVDEBUG("unhandled NavFuncQuery::status_t %d\n", status);
				glColor3d(0.5, 0.5, 0.5); // grey
      }
      glVertex2d(pt.v0, pt.v1);
    }
    glEnd();
	}

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}


void ArcDrawing::
KeyPressed(unsigned char key)
{
  if('o' == key)
    m_enabled = ! m_enabled;
}
