/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#include "RHDrawing.hpp"
#include "BLDrawing.hpp"
#include <iostream>


using sfl::ReplanHandler;
using sfl::BubbleList;
using namespace std;
using namespace npm;


RHDrawing::
RHDrawing(const string & name,
	  const ReplanHandler * replan_handler,
	  mode_t mode):
  Drawing(name,
	  "wrapped bubble band drawing, depending on planning state"),
  _replan_handler(replan_handler),
  _mode(mode)
{
}


void RHDrawing::
Draw()
{
  if((_mode == BAND) || (_mode == INITIALBAND))
    DrawBand();
  else if(_mode == AUTODETECT){
    // partly deprecated, used to draw NF1
    if( ! _replan_handler->InitialBand() ||
	_replan_handler->InitialBand()->Empty())
      return;
    BLDrawing("",_replan_handler->InitialBand(), BLDrawing::REDUCED).Draw();
  }
  else{
    cerr << "ERROR in RHDrawing::Draw()\n  invalid mode\n";
    exit(EXIT_FAILURE);
  }
}


void RHDrawing::
DrawBand()
{
  const BubbleList * bl;

  if(_mode == BAND)
    bl = _replan_handler->BufferBlist();
  else
    bl = _replan_handler->InitialBand();

  if( ! bl || bl->Empty())
    return;
  
  BLDrawing("", bl, BLDrawing::REDUCED).Draw();
}
