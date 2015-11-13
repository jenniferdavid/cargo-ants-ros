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


#ifndef NPM_CHEATSHEET_HPP
#define NPM_CHEATSHEET_HPP


#include <boost/shared_ptr.hpp>
#include <vector>


namespace npm {

  class World;
  class RobotServer;

  class CheatSheet
  {
  public:
    typedef struct { double x0, y0, x1, y1; } line_t;
    typedef struct { double x, y, r; } dynobj_t;
  
    const World * world;
    const RobotServer * robot;
  
    std::vector<line_t> line;
    std::vector<dynobj_t> dynobj;
  
    CheatSheet(const World * world, const RobotServer * robot);
  
    void UpdateLines();
    void UpdateDynobjs();
  };

}

#endif // NPM_CHEATSHEET_HPP
