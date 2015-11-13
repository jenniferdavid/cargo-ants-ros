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


#include "PathDrawing.hpp"
#include "Smart.hpp"
#include "../common/wrap_gl.hpp"
#include "../common/wrap_glu.hpp"
#include "../common/Manager.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <asl/path_tracking.hpp>
#include <math.h>

// debugging
#include <estar/pdebug.hpp>
#define PDEBUG PDEBUG_OFF
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace npm;
using namespace estar;
using namespace boost;


PathDrawing::
PathDrawing(const std::string & name,
	    const AslBot * aslbot,
	    size_t _gradplot_frequency)
  : Drawing(name,
	    "(current and maybe previous) planned path (gradient descent)",
	    Instance<UniqueManager<Drawing> >()),
    gradplot_frequency(_gradplot_frequency),
    m_aslbot(aslbot)
{
}


static void draw_path(const asl::path_t * path, size_t gradplot_frequency,
		      double color_intensity, double carrot_radius)
{
  if(( ! path) || (path->empty())){
    PDEBUG("invalid or empty path\n");
    return;
  }
  
  const double startval(path->front().value);
  double deltav(absval(startval - path->back().value));
  if(deltav <= epsilon)
    deltav = startval;
  
  if(0 == gradplot_frequency){
    glLineWidth(1);
    glBegin(GL_LINE_STRIP);
    for(size_t ii(0); ii < path->size(); ++ii){
      const double blue((startval - (*path)[ii].value) / deltav);
      if((*path)[ii].degenerate)
	glColor3d(color_intensity,
		  color_intensity * 0.5,
		  color_intensity * blue);
      else
	glColor3d(color_intensity * 0.5,
		  color_intensity,
		  color_intensity * blue);
      glVertex2d((*path)[ii].point.v0, (*path)[ii].point.v1);
    }
    glEnd();
  }
  else{
    glPointSize(3);
    glBegin(GL_POINTS);
    for(size_t ii(0); ii < path->size(); ii += gradplot_frequency){
      const double blue((startval - (*path)[ii].value) / deltav);
      if((*path)[ii].degenerate)
	glColor3d(color_intensity,
		  color_intensity * 0.5,
		  color_intensity * blue);
      else
	glColor3d(color_intensity * 0.5,
		  color_intensity,
		  color_intensity * blue);
      glVertex2d((*path)[ii].point.v0, (*path)[ii].point.v1);
    }
    glEnd();
    glPointSize(1);
    glLineWidth(1);
    glBegin(GL_LINES);
    for(size_t ii(0); ii < path->size(); ii += gradplot_frequency){
      const double blue((startval - (*path)[ii].value) / deltav);
      if((*path)[ii].degenerate)
	glColor3d(color_intensity,
		  color_intensity * 0.5,
		  color_intensity * blue);
      else
	glColor3d(color_intensity * 0.5,
		  color_intensity,
		  color_intensity * blue);
      glVertex2d((*path)[ii].point.v0, (*path)[ii].point.v1);
      glVertex2d((*path)[ii].point.v0 + (*path)[ii].gradient.v0,
		 (*path)[ii].point.v1 + (*path)[ii].gradient.v1);
    }
    glEnd();
  }
  
  double carx(path->back().point.v0);
  double cary(path->back().point.v1);
  glColor3d(1, 1, 0);
  glPushMatrix();
  glTranslated(carx, cary, 0);
  gluDisk(wrap_glu_quadric_instance(), carrot_radius, carrot_radius, 36, 1);
  glPopMatrix();
}


void PathDrawing::
Draw()
{
  boost::shared_ptr<asl::path_t> clean;
  boost::shared_ptr<asl::path_t> dirty;
  m_aslbot->CopyPaths(clean, dirty);
  draw_path(clean.get(), gradplot_frequency, 1, 0.2);
  draw_path(dirty.get(), 0, 0.5, 0.15);

  /* trajectory and reference point drawings */
  asl::path_point ref_point;
  if(m_aslbot->GetRefpoint(ref_point))
    {
      glPointSize(8);
      glBegin(GL_POINTS);
      glColor3d(0.0, 1.0, 0.0);
      glVertex2d(ref_point.v0, ref_point.v1);
      glEnd();
       
    }else{
      PDEBUG("No reference point could be queried!!!\n");
    }

  const asl::trajectory_t *current_traj=m_aslbot->GetTrajectory();
  if(current_traj && (current_traj->size()>1))
    {
      glLineWidth(1);
      glBegin(GL_LINE_STRIP);
      glColor3d(1.0, 1.0, 0.5);
      for(size_t ii(0); ii< current_traj->size();ii++){
	glVertex2d((*current_traj)[ii].v0, (*current_traj)[ii].v1);
      }
      glEnd();
    }
}

