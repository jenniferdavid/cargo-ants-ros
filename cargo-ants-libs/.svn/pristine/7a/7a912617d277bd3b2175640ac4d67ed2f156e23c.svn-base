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


#include "CheatSheet.hpp"
#include "World.hpp"
#include "RobotServer.hpp"
#include "Object.hpp"
#include <sfl/util/Line.hpp>


using namespace sfl;
using namespace boost;


namespace npm {


  CheatSheet::
  CheatSheet(const World * _world, const RobotServer * _robot)
    : world(_world),
      robot(_robot)
  {
  }

  void CheatSheet::
  UpdateLines()
  {
    line.clear();
    for(size_t io(0); io < world->m_object.size(); ++io)
      for(size_t il(0); il < world->m_object[io]->GetNlines(); ++il){
	shared_ptr<const Line> foo(world->m_object[io]->GetGlobalLine(il));
	line_t bar;
	bar.x0 = foo->X0();
	bar.y0 = foo->Y0();
	bar.x1 = foo->X1();
	bar.y1 = foo->Y1();
	line.push_back(bar);
      }
  }


  void CheatSheet::
  UpdateDynobjs()
  {
    dynobj.clear();
    for(size_t ir(0); ir < world->m_robot.size(); ++ir){
      if(world->m_robot[ir] == robot)
	continue;
      dynobj_t dd;
      dd.x = world->m_robot[ir]->GetPose().X() ;
      dd.y = world->m_robot[ir]->GetPose().Y();
      dd.r = world->m_robot[ir]->GetBody().GetRadius();
      dynobj.push_back(dd);
    }
  }
  
}
