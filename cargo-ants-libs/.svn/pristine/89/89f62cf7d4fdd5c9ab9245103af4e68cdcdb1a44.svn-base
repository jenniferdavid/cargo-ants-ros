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


#ifndef MP_DRAWING_HPP
#define MP_DRAWING_HPP


#include <npm/gfx/Drawing.hpp>


namespace sfl {
  class MotionPlanner;
}


class MPDrawing
  : public npm::Drawing
{
public:
  MPDrawing(const std::string & name,
	    const sfl::MotionPlanner & mp);
  
  virtual void Draw();
  
private:
  const sfl::MotionPlanner & m_mp;
};

#endif // MP_DRAWING_HPP
