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


#ifndef DWDRAWING_HPP
#define DWDRAWING_HPP


#include <npm/gfx/Drawing.hpp>


namespace sfl {
  class DynamicWindow;
}


class DWDrawing
  : public npm::Drawing
{
public:
  DWDrawing(const std::string & name,
	    const sfl::DynamicWindow & dwa);
  
  virtual void Draw();
  
private:
  const sfl::DynamicWindow & m_dwa;
};

#endif // DWDRAWING_HPP
