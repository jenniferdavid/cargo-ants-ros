/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#ifndef NPM_MAPPER_UPDATE_DRAWING_HPP
#define NPM_MAPPER_UPDATE_DRAWING_HPP


#include <npm/gfx/Drawing.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {
  class Mapper2d;
}


namespace npm {
  
  
  class MapperUpdateDrawing
    : public Drawing
  {
  public:
		typedef enum {
			FREESPACE,
			OBSTACLE,
			CHECK,
			HOLE,
			REPAIR
		} what_t;
		
    MapperUpdateDrawing(const std::string & name,
												what_t what,
												boost::shared_ptr<const sfl::Mapper2d> mapper);
    
    virtual void Draw();
		
		what_t what;
		
  private:
    boost::shared_ptr<const sfl::Mapper2d> m_mapper;
  };

}

#endif // NPM_MAPPER_UPDATE_DRAWING_HPP
