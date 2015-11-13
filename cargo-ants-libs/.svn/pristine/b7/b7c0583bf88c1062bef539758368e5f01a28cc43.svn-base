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


#include "BLDrawing.hpp"
#include <npm/gfx/wrap_glu.hpp>
#include <sfl/bband/Bubble.hpp>
#include <iostream>


using sfl::Bubble;
using namespace std;
using namespace npm;


BLDrawing::
BLDrawing(const std::string & name,
	    const sfl::BubbleList * bubble_list,
	    mode_t mode,
	    double intensity):
  Drawing(name,
	  "sfl::BubbleList in global frame"),
  _bubble_list(bubble_list),
  _mode(mode),
  _intensity(intensity)
{
}


void BLDrawing::
Draw()
{
  switch(_mode){
  case FULL:
    DrawFull();
    break;
  case REDUCED:
    DrawReduced();
    break;
  case SNAPPED:
    DrawSnapped();
    break;
  default:
    cerr << "ERROR in BLDrawing::Draw()\n   invalid mode " << _mode << "\n";
    exit(EXIT_FAILURE);
  }
}



void BLDrawing::
DrawFull()
{
  glMatrixMode(GL_MODELVIEW);
  glPolygonMode(GL_FRONT, GL_FILL);

  static const double light(0.6);
  static const double dark(0.4);

  glColor4d(dark, dark, dark, 0.9);
  for(const Bubble * current(_bubble_list->Head());
      current != 0;
      current = current->Next()){

    glPushMatrix();
    glTranslated(current->X(), current->Y(), 0);
    gluDisk(wrap_glu_quadric_instance(),
	    0, current->IgnoreDistance(), 36, 1);
    glPopMatrix();
  }

  glColor4d(light, light, light, 0.9);
  for(const Bubble * current(_bubble_list->Head());
      current != 0;
      current = current->Next()){

    glPushMatrix();
    glTranslated(current->X(), current->Y(), 0);
    gluDisk(wrap_glu_quadric_instance(),
	    current->IgnoreDistance(), current->Radius(), 36, 1);
    glPopMatrix();
  }

  DrawReduced();
}



void BLDrawing::
SetIntensity(double i)
{
  _intensity = i;
}



void BLDrawing::
DrawReduced()
{
  glColor3d(_intensity, _intensity, _intensity);
  glLineWidth(1);
  glBegin(GL_LINE_STRIP);
  const Bubble *current(_bubble_list->Head());
  while(current != 0){
    glVertex2d(current->X(), current->Y());
    current = current->Next() ;
  }
  glEnd();
}



void BLDrawing::
DrawSnapped()
{
  glMatrixMode(GL_MODELVIEW);
  glLineWidth(1);
  glPolygonMode(GL_FRONT, GL_LINE);
  glColor3d(0.5, 0.5, 0.5);
    
  const Bubble * current(_bubble_list->Head());
  while(current != 0){
    glPushMatrix();
    
    glTranslated(current->X(), current->Y(), 0);
    gluDisk(wrap_glu_quadric_instance(),
	    current->Radius(), current->Radius(), 36, 1);
    
    glPopMatrix();
    
    current = current->Next();
  }
  
  DrawReduced();
}
