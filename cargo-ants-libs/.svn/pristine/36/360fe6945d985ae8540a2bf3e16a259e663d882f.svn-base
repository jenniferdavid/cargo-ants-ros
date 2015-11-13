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


#ifndef PNF_DRAWING_HPP
#define PNF_DRAWING_HPP


#include <npm/gfx/Drawing.hpp>


namespace npm {

class Esbot;


class PNFDrawing
  : public npm::Drawing
{
public:
  typedef enum { GLOBAL, LOCAL, GFRAME } mode_t;
  typedef enum { RISK, VALUE, META, AUTO } what_t;
  
  mode_t mode;
  what_t what;
  bool draw_trace;		// false by default
  
  PNFDrawing(const std::string & name,
	     Esbot * bot,
	     mode_t mode,
	     what_t what);
  
  virtual void Draw();
  
private:
  Esbot * m_bot;
};

}

#endif // PNF_DRAWING_HPP
