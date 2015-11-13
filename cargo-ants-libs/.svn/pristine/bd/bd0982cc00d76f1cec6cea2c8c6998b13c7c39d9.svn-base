/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "GridLayerDrawing.hpp"
#include <npm/gfx/wrap_gl.hpp>
#include <sfl/gplan/NF1.hpp>
#include <sfl/gplan/NF1Wave.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <iostream>


using namespace sfl;
using namespace boost;
using namespace std;
using namespace npm;


GridLayerDrawing::
GridLayerDrawing(const string & name, const NF1 & _nf1, bool _global)
  : Drawing(name,
	    "greyscale + special color grid plot, local or global frame"),
    nf1(_nf1), global(_global)
{
}


/**
   \note should not hardcode special colors...
   \todo use GLU instead of transforming 4 points per cell.
*/
void GridLayerDrawing::
Draw()
{
  shared_ptr<const NF1::grid_t> layer(nf1.GetGridLayer());
  if( ! layer->ValidIndex(static_cast<size_t>(0), static_cast<size_t>(0)))
    return;
  double scale((*layer)[0][0]);
  for(size_t ii(0); ii < layer->xsize; ++ii)
    for(size_t jj(0); jj < layer->ysize; ++jj)
      if((*layer)[ii][jj] > scale)
	scale = (*layer)[ii][jj];
  if(scale > 0)
    scale = 1 / scale;
  else
    scale = 1;
  
  if(global){
    shared_ptr<const GridFrame> gf(nf1.GetGridFrame());
    typedef NF1::position_t p_t;
    const p_t hex((gf->GlobalPoint(0, 1) - gf->GlobalPoint(0, 0)) / 2.0);
    const p_t hey((gf->GlobalPoint(1, 0) - gf->GlobalPoint(0, 0)) / 2.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    for(size_t ii(0); ii < layer->xsize; ++ii)
      for(size_t jj(0); jj < layer->ysize; ++jj){
	if(NF1Wave::FREE == (*layer)[ii][jj])
	  glColor3d(0, 0, 1);
	else if(NF1Wave::OBSTACLE == (*layer)[ii][jj])
	  glColor3d(1, 0, 0);
	else{
	  double grey((*layer)[ii][jj] * scale);
	  glColor3d(grey, grey, grey);
	}
	const p_t p0(gf->GlobalPoint(ii, jj));
	const p_t p1(p0 - hex - hey);
	const p_t p2(p0 + hex - hey);
	const p_t p3(p0 + hex + hey);
	const p_t p4(p0 - hex + hey);
	glBegin(GL_POLYGON);
	glVertex2d(p1.v0, p1.v1);
	glVertex2d(p2.v0, p2.v1);
	glVertex2d(p3.v0, p3.v1);
	glVertex2d(p4.v0, p4.v1);
	glEnd();
      }
  }
  else{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    for(size_t ii(0); ii < layer->xsize; ++ii)
      for(size_t jj(0); jj < layer->ysize; ++jj){
	if(NF1Wave::FREE == (*layer)[ii][jj])
	  glColor3d(0, 0, 1);
	else if(NF1Wave::OBSTACLE == (*layer)[ii][jj])
	  glColor3d(1, 0, 0);
	else{
	  double grey((*layer)[ii][jj] * scale);
	  glColor3d(grey, grey, grey);
	}
	glRectd(ii, jj, ii + 1, jj + 1);
      }
  }
}
