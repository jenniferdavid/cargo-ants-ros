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


#include "BBDrawing.hpp"
#include "BLDrawing.hpp"
#include <npm/gfx/wrap_gl.hpp>
#include <iostream>


using sfl::BubbleBand;
using namespace std;
using namespace npm;


BBDrawing::
BBDrawing(const std::string & name,
	  const BubbleBand & bubble_band,
	  mode_t mode)
  : Drawing(name,
	    "sfl::BubbleBand in global frame"),
    _bubble_band(bubble_band),
    _mode(mode)
{
}



void BBDrawing::
Draw()
{
  if(_mode == AUTODETECT)
    DrawAutodetect();
  else if((_mode == FULL) ||
	  (_mode == REDUCED))
    DrawUsual();
  else{
    cerr << "ERROR in BBDrawing::Draw()\n   invalid mode " << _mode << "\n";
    exit(EXIT_FAILURE);
  }
}


/**
   \todo Lots of temporary objects!
*/
void BBDrawing::DrawAutodetect()
{
  switch(_bubble_band.GetState()){
  case BubbleBand::NOBAND:
    // draw a line from robot to goal
    glColor3d(0.5, 0.5, 0.5);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex2d(_bubble_band.RobotPose().X(), _bubble_band.RobotPose().Y());
    glVertex2d(_bubble_band.GlobalGoal().X(), _bubble_band.GlobalGoal().Y());
    glEnd();
    break;
  case BubbleBand::VALIDBAND:
    BLDrawing("", _bubble_band.ActiveBlist(), BLDrawing::FULL).Draw();
    break;
  case BubbleBand::UNSUREBAND:
    BLDrawing("", _bubble_band.ActiveBlist(), BLDrawing::SNAPPED).Draw();
    break;
  default:
    cerr << "ERROR in BBDrawing::DrawAutodetect()\n"
	 << "   invalid state " << _bubble_band.GetState()
	 << " of BubbleBand\n";
    exit(EXIT_FAILURE);
  }
}



void BBDrawing::DrawUsual()
{
  if( ! _bubble_band.ActiveBlist() ||
      _bubble_band.ActiveBlist()->Empty())
    return;

  BLDrawing *foo;
  if(_mode == FULL)
    foo = new BLDrawing("", _bubble_band.ActiveBlist());
  else
    foo = new BLDrawing("", _bubble_band.ActiveBlist(), BLDrawing::REDUCED);
  foo->Draw();
  delete foo;
}
