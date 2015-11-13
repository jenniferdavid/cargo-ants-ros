/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "EstarDrawing.hpp"
#include <npm/gfx/wrap_gl.hpp>
#include <npm/gfx/View.hpp>
#include <npm/BBox.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <estar/graphics.hpp>
#include <estar/util.hpp>
#include <estar/Facade.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>


#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace boost;
using namespace std;

namespace npm {


EstarDrawing::
EstarDrawing(const std::string & name,
						 shared_ptr<PlanProxy> proxy,
						 what_t _what)
  : Drawing(name,
						"default color E* (VALUE/RHS/META/QUEUE/UPWIND/OBST/STATUS)"),
    what(_what),
    m_proxy(proxy)
{
}


EstarDrawing::
EstarDrawing(const std::string & name,
						 shared_ptr<PlanProxy> proxy,
						 what_t _what,
						 shared_ptr<gfx::ColorScheme> custom_cs)
  : Drawing(name,
						"custom color E* grid (VALUE/RHS/META/QUEUE/UPWIND/OBST/STATUS)"),
    what(_what),
    m_proxy(proxy),
    m_custom_cs(custom_cs)
{
}


void EstarDrawing::
Draw()
{
  if( ! m_proxy->Enabled())
    return;
  
  const estar::Facade * facade(m_proxy->GetFacade());
  const sfl::GridFrame * gframe(m_proxy->GetFrame());
  if(( ! facade) || ( ! gframe))
    return;
  
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(gframe->X(), gframe->Y(), 0);
  glRotated(180 * gframe->Theta() / M_PI, 0, 0, 1);
  glScaled(gframe->Delta(), gframe->Delta(), 1);
  
  const gfx::ColorScheme * cs;
	bool autoscale_value;
  if(m_custom_cs){
    cs = m_custom_cs.get();
		autoscale_value = false;
  }
  else{
    cs = gfx::ColorScheme::Get(gfx::GREY_WITH_SPECIAL);
		autoscale_value = true;
  }
  
  switch(what){
  case VALUE:
    gfx::draw_grid_value(*facade, cs, autoscale_value);
    break;
  case RHS:
    gfx::draw_grid_rhs(*facade, cs);
    break;
  case META:
    gfx::draw_grid_meta(*facade, cs);
    break;
  case QUEUE:
    gfx::draw_grid_queue(*facade);
    break;
  case UPWIND:
    gfx::draw_grid_upwind(*facade, 1, 0, 0, 2, true);
    break;
  case OBST:
    gfx::draw_grid_obstacles(*facade, 1, 0.5, 0.5);
    break;
  case STATUS:
    gfx::draw_grid_status(*facade);
    break;
  default:
    cerr << "ERROR in EstarDrawing::Draw(): invalid what=" << what
	 << " (expected VALUE, META, QUEUE, UPWIND, or STATUS)\n";
    exit(EXIT_FAILURE);
  }
  
	//// yellow bounding box
	//   double x0, y0, x1, y1;
	//   gfx::get_grid_bbox(*facade, x0, y0, x1, y1);
	//   glColor3d(1, 1, 0);
	//   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//   glRectd(x0, y0, x1, y1);
  
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}


EstarCamera::
EstarCamera(const std::string & name,
						boost::shared_ptr<PlanProxy> proxy)
	: Camera(name,
					 "bbox of E* C-space"),
		m_proxy(proxy)
{
}


void EstarCamera::
ConfigureView(View & view)
{
	estar::Facade const * facade(m_proxy->GetFacade());
	sfl::GridFrame const * gframe(m_proxy->GetFrame());
  if (( ! facade) || ( ! gframe)) {
		view.SetBounds(0, 0, 1, 1);
		return;
	}
	
	double x0, y0, x1, y1;
  gfx::get_grid_bbox(*facade, x0, y0, x1, y1);
	double const scale(gframe->Delta());
	x0 *= scale;
	y0 *= scale;
	x1 *= scale;
	y1 *= scale;
	double x2(x0);
	double y2(y1);
	double x3(x1);
	double y3(y0);
	gframe->To(x0, y0);
	gframe->To(x1, y1);
	gframe->To(x2, y2);
	gframe->To(x3, y3);
	BBox bb(x0, y0, x1, y1);
	bb.Update(x2, y2);
	bb.Update(x3, y3);
	view.SetBounds(bb);
}

}
