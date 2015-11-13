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


#include "ODrawing.hpp"
#include <npm/gfx/wrap_gl.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/Objective.hpp>
#include <sfl/util/numeric.hpp>


using namespace npm;


ODrawing::
ODrawing(const std::string & name,
	 boost::shared_ptr<sfl::Objective const> obj,
	 boost::shared_ptr<sfl::DynamicWindow const> dwa)
  : Drawing(name,
	    "a DWA's sub-objective (greyscale + special colors)"),
    m_obj(obj),
    m_dwa(dwa)
{
}


void ODrawing::
Draw()
{
  int qdlMin, qdlMax, qdrMin, qdrMax;
  double funcMin, scale;
  if (m_obj->UsesEntireVelocitySpace()) {
    qdlMin = 0;
    qdlMax = m_obj->dimension - 1;
    qdrMin = 0;
    qdrMax = m_obj->dimension - 1;
    scale = 1.0 / (m_obj->maxValue - m_obj->minValue);
    funcMin = m_obj->minValue;
  }
  else {
    qdlMin = m_dwa->QdlMinIndex();
    if (qdlMin < 0)
      return;
    qdlMax = m_dwa->QdlMaxIndex();
    qdrMin = m_dwa->QdrMinIndex();
    qdrMax = m_dwa->QdrMaxIndex();
    funcMin = m_obj->Min(qdlMin, qdlMax, qdrMin, qdrMax);
    double const funcMax(m_obj->Max(qdlMin, qdlMax, qdrMin, qdrMax));
    if (funcMax - funcMin < sfl::epsilon)
      scale = 0;
    else
      scale = 1 / (funcMax - funcMin);
  }
  
  glPolygonMode(GL_FRONT, GL_FILL);
  double grey;
  for(int l = qdlMin; l <= qdlMax; ++l)
    for(int r = qdrMin; r <= qdrMax; ++r){
      if( ! m_dwa->Forbidden(l, r)){
	grey = scale * (m_obj->Value(l, r) - funcMin);
	glColor3d(grey, grey, grey);
      }
      else
	glColor3d(0.7, 0, 0);
      
      glRectd(l, r, l + 1, r + 1);
      
      if(m_dwa->Reachable(l, r)){
	glColor3d(1, 0, 0);
	glLineWidth(1);
	glBegin(GL_LINES);
	glVertex2d(l,   r);
	glVertex2d(l+1, r+1);
	glVertex2d(l,   r+1);
	glVertex2d(l+1, r);
	glEnd();
      }
    }
  
  if(m_dwa->QdlOptIndex() >= 0){
    const int qdlOpt(m_dwa->QdlOptIndex());
    const int qdrOpt(m_dwa->QdrOptIndex());
    glLineWidth(1);
    glPolygonMode(GL_FRONT, GL_LINE);
    glColor3d(0, 1, 1);
    glRectd(qdlOpt, qdrOpt, qdlOpt + 1, qdrOpt + 1);
  }
}
