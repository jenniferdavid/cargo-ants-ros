/* 
 * Copyright (C) 2005
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


#include "SequencerDrawing.hpp"
#include "TheaterRobot.hpp"
#include <theater/Sequencer.hpp>
#include <theater/Behavior.hpp>


namespace SequencerDrawing {


  Stage::
  Stage(const std::string & name,
	TheaterRobot & owner):
    Drawing(name),
    _owner(owner)
  {
  }
  
  
  void Stage::
  Draw()
  {
    _owner.GetSequencer()->Draw();
  }
  
}
