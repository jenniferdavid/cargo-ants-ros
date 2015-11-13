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


#include "PNFDrawing.hpp"
#include "PNF.hpp"
#include "Esbot.hpp"
#include <npm/gfx/wrap_gl.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <estar/graphics.hpp>
#include <estar/util.hpp>
#include <estar/pdebug.hpp>
#include <estar/Facade.hpp>
#include <pnf/Flow.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <cmath>

// for boost::tie
#include <boost/graph/adjacency_list.hpp>


#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace npm;
using namespace boost;


namespace local {
  
  class ValueColorScheme
    : public gfx::ColorScheme {
  public:
    ValueColorScheme(double _vmax): vmax(_vmax) {
      PVDEBUG("vmax = %g\n", vmax);
    }
    
    virtual void Set(double value) const {
      double red, green, blue;
      if(absval(value - vmax) < 0.3)
	red = 0.6;
      else
	red = boundval(0.0, 0.5 * (value / vmax), 0.5);
      if(absval(value) < 0.3)
	green = 0.6;
      else
	green = boundval(0.0, 0.5 * ((vmax - value) / vmax), 0.5);
      if(absval(value - vmax / 2) < 0.3)
	blue = 0.6;
      else
	blue = 0;
      glColor3d(red, green, blue);
    }
    const double vmax;
  };
  
}

using namespace local;


PNFDrawing::
PNFDrawing(const std::string & name,
	   Esbot * bot,
	   mode_t _mode,
	   what_t _what)
  : Drawing(name,
	    "the PNF of an Esbot instance (risk, value, meta, or auto)"),
    mode(_mode),
    what(_what),
    draw_trace(false),
    m_bot(bot)
{
}


void PNFDrawing::
Draw()
{
  boost::shared_ptr<PNF> pnf(m_bot->GetPNF());
  if( ! pnf)
    return;
  
  const mode_t mm(mode);	// multithread-paranoia
  if(GFRAME != mm){
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    if(GLOBAL == mm){
      shared_ptr<const sfl::Frame> frame(pnf->GetGridFrame());
      glTranslated(frame->X(), frame->Y(), 0);
      glRotated(180 * frame->Theta() / M_PI, 0, 0, 1);
    }
    glScaled(pnf->resolution, pnf->resolution, 1);
  }
  
  const PNF::step_t step(pnf->GetStep(false));
  boost::shared_ptr<pnf::Flow> flow(pnf->GetFlow());
  const estar::Facade * facade(0);
  const gfx::ColorScheme * cs(gfx::ColorScheme::Get(gfx::GREY_WITH_SPECIAL));
  
  if(RISK == what){
    if(PNF::DONE == step){
      facade = &flow->GetPNF();
      const estar::array<double> * risk;
      double max_risk;
      boost::tie(risk, max_risk) = flow->GetRisk();
      gfx::draw_array(*risk, 0, 0, flow->xsize - 1, flow->ysize - 1,
		      0, max_risk, gfx::ColorScheme::Get(gfx::BLUE_GREEN_RED));
    }
  }
  else if(VALUE == what){
    if(PNF::DONE == step){
      facade = &flow->GetPNF();
      shared_ptr<const GridFrame> gframe(pnf->GetGridFrame());
      sfl::Pose pp;
      m_bot->GetPose(pp);
      double x = pp.X();
      double y = pp.Y();
      gframe->From(x, y);
      const size_t ix(static_cast<size_t>(rint(x / facade->scale)));
      const size_t iy(static_cast<size_t>(rint(y / facade->scale)));
      if ( ! facade->IsValidIndex(ix, iy)) {
	PDEBUG("FAIL facade->IsValidIndex(ix, iy)\n");
	return;
      }
      const ValueColorScheme vcs(facade->GetValue(ix, iy));
      gfx::draw_grid_value(*facade, &vcs, false);
    }
  }
  else if(META == what){
    if(PNF::DONE == step){
      facade = &flow->GetPNF();
      gfx::draw_grid_meta(*facade, cs);
    }
  }
  else if(AUTO == what){
    switch(step){
    case PNF::NONE:
      facade = &flow->GetEnvdist();
      break;
    case PNF::ENVDIST:
    case PNF::OBJDIST:
      facade = &flow->GetEnvdist();
      gfx::draw_grid_value(*facade, cs, true); // should use custom cs
      break;
    case PNF::ROBDIST:
      facade = flow->GetRobdist(); // could be 0
      if(0 != facade)
	gfx::draw_grid_value(*facade, cs, true); // should use custom cs
      break;
    case PNF::DONE:
      facade = &flow->GetPNF();
      gfx::draw_grid_meta(*facade, cs);
      break;
    default:
      std::cerr << "BUG in PNFDrawing::Draw(): unhandled step "
		<< step << "in what == AUTO\n";
      exit(EXIT_FAILURE);
    }
  }
  else{
    std::cerr << "BUG in PNFDrawing::Draw(): invalid what " << what << "\n";
    exit(EXIT_FAILURE);
  }
  
  if(0 != facade){
    double x0, y0, x1, y1;
    gfx::get_grid_bbox(*facade, x0, y0, x1, y1);
    glColor3d(1, 1, 0);
    glLineWidth(1);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glRectd(x0, y0, x1, y1);
  }
  
  if(GFRAME != mm){
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }
  
  if(draw_trace && (PNF::DONE == step)){
    shared_ptr<const sfl::Frame> frame(pnf->GetGridFrame());
    double robx(pnf->robot_x);
    double roby(pnf->robot_y);
    frame->From(robx, roby);
    gfx::draw_trace(*facade, robx, roby,
		    gfx::ColorScheme::Get(gfx::GREEN_PINK_BLUE),
		    1, 0.5, 0);
  }
}
