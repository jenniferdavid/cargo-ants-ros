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


#include "DWDrawing.hpp"
#include <npm/gfx/wrap_gl.hpp>
#include <sfl/dwa/DynamicWindow.hpp>


using namespace npm;


DWDrawing::
DWDrawing(const std::string & name,
	  const sfl::DynamicWindow & dwa)
  : Drawing(name,
	    "an grid-overview of DWA: objectives, speed range, and optimum"),
    m_dwa(dwa)
{
}



void DWDrawing::
Draw()
{
  const int qdlMin(m_dwa.QdlMinIndex());
  if(qdlMin < 0)
    return;
  
  const int qdlMax(m_dwa.QdlMaxIndex());
  const int qdrMin(m_dwa.QdrMinIndex());
  const int qdrMax(m_dwa.QdrMaxIndex());
  const int qdlOpt(m_dwa.QdlOptIndex());
  const int qdrOpt(m_dwa.QdrOptIndex());
  const int dimension(m_dwa.Dimension());
  const double objMax(m_dwa.ObjectiveMax());
  const double objMin(m_dwa.ObjectiveMin());
  const double scale(1 / (objMax - objMin));

  glPolygonMode(GL_FRONT, GL_FILL);

  for(int il = 0; il < qdlMin; ++il)
    for(int ir = 0; ir < dimension; ++ir){
      if(m_dwa.Forbidden(il, ir))
	glColor3d(0.4, 0, 0);
      else
	glColor3d(0.7, 0, 0);
      glRectd(il, ir, il + 1, ir + 1);
    }

  for(int il = qdlMax + 1; il < dimension; ++il)
    for(int ir = 0; ir < dimension; ++ir){
      if(m_dwa.Forbidden(il, ir))
	glColor3d(0.4, 0, 0);
      else
	glColor3d(0.7, 0, 0);
      glRectd(il, ir, il + 1, ir + 1);
    }

  for(int il = qdlMin; il <= qdlMax; ++il){
    for(int ir = 0; ir < qdrMin; ++ir){
      if(m_dwa.Forbidden(il, ir))
	glColor3d(0.4, 0, 0);
      else
	glColor3d(0.7, 0, 0);
      glRectd(il, ir, il + 1, ir + 1);
    }

    for(int ir = qdrMax + 1; ir < dimension; ++ir){
      if(m_dwa.Forbidden(il, ir))
	glColor3d(0.4, 0, 0);
      else
	glColor3d(0.7, 0, 0);
      glRectd(il, ir, il + 1, ir + 1);
    }

    for(int ir = qdrMin; ir <= qdrMax; ++ir){
      if(m_dwa.Admissible(il, ir)){
	double grey(scale * (m_dwa.GetObjectiveSum(il, ir) - objMin));
	glColor3d(grey, grey, grey);
      }
      else if(m_dwa.Forbidden(il, ir))
	glColor3d(0.4, 0, 0);
      else
	glColor3d(1, 0, 0);
      glRectd(il, ir, il + 1, ir + 1);
    }
  }

  if(qdlOpt >= 0){
    // highlight max
    glLineWidth(1);
    glPolygonMode(GL_FRONT, GL_LINE);
    glColor3d(0, 1, 1);
    glRectd(qdlOpt, qdrOpt, qdlOpt + 1, qdrOpt + 1);
  }
  
}
