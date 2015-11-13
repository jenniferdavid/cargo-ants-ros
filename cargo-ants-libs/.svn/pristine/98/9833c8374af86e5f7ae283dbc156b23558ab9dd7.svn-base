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


#include "World.hpp"
#include "Object.hpp"
#include "RobotServer.hpp"
#include "gfx/WorldDrawing.hpp"
#include "gfx/WorldCamera.hpp"
#include "BBox.hpp"
#include "pdebug.hpp"
#include <sfl/util/pdebug.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/Random.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <sstream>
#include <cmath>
#include <err.h>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  World::
  World(const string & name)
    : fpplib::Configurable(name),
      m_drawing(new WorldDrawing(name, * this)),
      m_camera(new WorldCamera(name, * this))
  {
    m_object.push_back(shared_ptr<Object>(new Object(name, "THE world")));
    reflectCallback<string> ("builtin", false, boost::bind(&World::LoadBuiltin, this, _1));
    reflectCallback<Line> ("lines", true, boost::bind(&World::AddLine, this, _1));
  }
  
  
  bool World::
  LoadBuiltin(const string & name)
  {
    if(name == "mini")
      LoadMini();
    else if(name == "expo")
      LoadExpo();
    else if(name == "stage")
      LoadStage();
    else
      return false;
    return true;
  }
  
  
  void World::
  LoadMini()
  {
    AddLine(Line(0, 3, 1, 3));
    AddLine(Line(-1, -1, 4, -1));
    AddLine(Line(4, -1, 4, 4));
    AddLine(Line(4, 4, -1, 4));
    AddLine(Line(-1, 4, -1, -1));
  }


  void World::
  LoadExpo()
  {
    AddLine(Line( 0  ,  8  , 12  ,  3  ));
    AddLine(Line(12  ,  3  , 24  ,  2.3));
    AddLine(Line(24  ,  2.3, 18  , 24  ));
    AddLine(Line(18  , 24  , 18  , 28.9));
    AddLine(Line( 9.4, 27.7, 15.1, 28.9));
    AddLine(Line(15.1, 28.9, 14  , 24  ));
    AddLine(Line(14  , 24  , 10  , 24  ));
    AddLine(Line(10  , 24  ,  4.6, 21.3));
    AddLine(Line( 4.6, 21.3,  2.6, 22.4));
    AddLine(Line( 2.6, 22.4,  1.4, 18  ));
    AddLine(Line( 1.4, 18  ,  3.6, 18  ));
    AddLine(Line( 3.6, 18  ,  6  , 12  ));
    AddLine(Line( 6  , 12  , 11.4,  9.9));
    AddLine(Line(11.4,  9.9, 11  ,  8.6));
    AddLine(Line(11  ,  8.6,  5.1, 10.3));

    Object column("column", "");
    column.AddLine(Line(-0.2, -0.2,  0.2, -0.2));
    column.AddLine(Line( 0.2, -0.2,  0.2,  0.2));
    column.AddLine(Line( 0.2,  0.2, -0.2,  0.2));
    column.AddLine(Line(-0.2,  0.2, -0.2, -0.2));
    for(int x(6); x <= 18; x += 6)
      for(int y(6); y <= 18; y += 6){
	Object col(column);
	col.TransformTo(Frame(x, y, 0));
	AddObject(col);
      }

    Object biotop("biotop", "");
    biotop.AddLine(Line(-1.3, 0, 0, -1.3));
    biotop.AddLine(Line(0, -1.3, 3.6, 0));
    biotop.AddLine(Line(3.6, 0, 0, 1.3));
    biotop.AddLine(Line(0, 1.3, -1.3, 0));
    biotop.TransformTo(Frame(12, 16.3, 7 * M_PI / 180));
    AddObject(biotop);
  }
  
  
  bool World::
  AddLine(const Line & line)
  {
    m_object[0]->AddLine(line);
    if( ! m_bbox)
      m_bbox.reset(new BBox(line));
    else
      m_bbox->Update(line);
    return true;
  }
  
  
  void World::
  AddObject(const Object & object)
  {
    m_object.push_back(shared_ptr<Object>(new Object(object)));
    if( ! m_bbox)
      m_bbox.reset(new BBox(*object.GetGlobalBB()));
    else
      m_bbox->Update(*object.GetGlobalBB());
  }
  
  
  void World::
  UpdateSensor(Sensor & sensor) const
  {
    for(size_t io(0); io < m_object.size(); ++io)
      m_object[io]->UpdateSensor(sensor);
    for(size_t ir(0); ir < m_robot.size(); ++ir)
      m_robot[ir]->UpdateSensor(sensor);
  }
  
  
  void World::
  AddRobot(RobotServer * robot)
  {
    m_robot.push_back(robot);
  }
  
  
  void World::
  LoadStage()
  {
    //////////////////////////////////////////////////
    // H columns
    {
      Object column("H column", "");
      
      static const double width(0.303);
      static const double height(0.15);
      static const double thick(0.02);
      static const double empty(0.065);
    
      // bottom hollow
      column.AddLine(Line(0,             0,      thick,         0));
      column.AddLine(Line(thick,         0,      thick,         empty));
      column.AddLine(Line(thick,         empty,  width - thick, empty));
      column.AddLine(Line(width - thick, empty,  width - thick, 0));
      column.AddLine(Line(width - thick, 0,      width,         0));

      // top hollow
      column.AddLine(Line(0,           height,       thick,     height));
      column.AddLine(Line(thick,       height,       thick,     height-empty));
      column.AddLine(Line(thick,     height-empty, width-thick, height-empty));
      column.AddLine(Line(width-thick, height-empty, width-thick, height));
      column.AddLine(Line(width-thick, height,        width,      height));

      // sides
      column.AddLine(Line(0,     0,     0, height));
      column.AddLine(Line(width, 0, width, height));

      Frame pos[14];
    
      pos[ 0] = Frame(0.0000, 0.0000, 0);
      pos[ 1] = Frame(0.0000, 1.9950, 0);
      pos[ 2] = Frame(0.0000, 4.0650, 0);
      pos[ 3] = Frame(0.0000, 6.1350, 0);
    
      pos[ 4] = Frame(8.5730, 0.0000, 0);
      pos[ 5] = Frame(8.5730, 1.9950, 0);
      pos[ 6] = Frame(8.5730, 4.0650, 0);
      pos[ 7] = Frame(8.5730, 6.1350, 0);
    
      pos[ 8] = Frame(2.1085, 7.6370, M_PI / 2);
      pos[ 9] = Frame(6.8425, 7.6370, M_PI / 2);
      pos[10] = Frame(3.9405, 7.6370, M_PI / 2);
      pos[11] = Frame(5.0855, 7.6370, M_PI / 2);
      pos[12] = Frame(3.9405, 6.1370, M_PI / 2);
      pos[13] = Frame(5.0855, 6.1370, M_PI / 2);
    
      for(int i(0); i < 14; ++i){
	Object col(column);
	col.TransformTo(pos[i]);
	AddObject(col);
      }
    }
  
    //////////////////////////////////////////////////
    // round columns
    {
      Object column("round column", "");
    
      static const double radius(0.04);
      static const int nlines(16);
      static const double dtheta(2 * M_PI / nlines);
      for(int i(0); i < nlines; ++i)
	column.AddLine(Line(radius * cos(i * dtheta),
			    radius * sin(i * dtheta),
			    radius * cos(i * dtheta + dtheta),
			    radius * sin(i * dtheta + dtheta)));
    
      Frame pos[7];
    
      pos[ 0] = Frame(1.4970, 6.2100, 0);
      pos[ 1] = Frame(6.1320, 6.2100, 0);
      pos[ 2] = Frame(7.3790, 6.2100, 0);

      pos[ 3] = Frame(2.7440, 4.9210, 0);
      pos[ 4] = Frame(6.1320, 4.9210, 0);

      pos[ 5] = Frame(2.0330, 4.2090, 0);
      pos[ 6] = Frame(6.8430, 4.2090, 0);
    
      for(int i(0); i < 7; ++i){
	Object col(column);
	col.TransformTo(pos[i]);
	AddObject(col);
      }
    }

    //////////////////////////////////////////////////  
    // square columns
    {
      Object column("square column", "");
    
      static const double width(0.25);
      column.AddLine(Line(    0,     0, width,     0));
      column.AddLine(Line(width,     0, width, width));
      column.AddLine(Line(width, width,     0, width));
      column.AddLine(Line(    0, width,     0,     0));

      Frame pos[2];
      pos[ 0] = Frame(0.0000, 7.6900, 0);
      pos[ 1] = Frame(8.6260, 7.6900, 0);
    
      for(int i(0); i < 2; ++i){
	Object col(column);
	col.TransformTo(pos[i]);
	AddObject(col);
      }
    }
  
    //////////////////////////////////////////////////
    // walls
  
    AddLine(Line(0.0000, 0.0000, 0.0000, 7.9400));
    AddLine(Line(8.8760, 0.0000, 8.8760, 7.9400));
    AddLine(Line(0.0000, 7.9400, 8.8760, 7.9400));
  }
  
  
  void World::
  DumpLines(ostream & os, bool use_windows_eol) const
  {
    string eol;
    if(use_windows_eol)
      eol = "\r\n";
    else
      eol = "\n";
  
    os << "# map of \"" << m_object[0]->name
       << "\": floor x0 y0 x1 y1" << eol
       << "1" << eol
       << "1\tfloor" << eol;
    for(size_t io(0); io < m_object.size(); ++io)
      for(size_t il(0); il < m_object[io]->GetNlines(); ++il){
	shared_ptr<const Line> foo(m_object[io]->GetGlobalLine(il));
	os << "1\t" << foo->X0() << "\t" << foo->Y0()
	   << "\t" << foo->X1() << "\t" << foo->Y1() << eol;
      }
  }
  
  
  void World::
  ApplyTraversability(const TraversabilityMap & travmap, bool simple)
  {
    GridFrame const & gframe(travmap.gframe);
    double const offset(gframe.Delta() / 2);
    int const obstacle(travmap.obstacle);
    ssize_t const xbegin(travmap.grid.xbegin());
    ssize_t const xend(travmap.grid.xend());
    ssize_t const ybegin(travmap.grid.ybegin());
    ssize_t const yend(travmap.grid.yend());
    
    for (ssize_t ix(xbegin); ix < xend; ++ix)
      for (ssize_t iy(ybegin); iy < yend; ++iy)
	if (travmap.grid.at(ix, iy) >= obstacle) {
	  GridFrame::position_t center(gframe.LocalPoint(ix, iy));
	  if (simple) {
	    {
	      Line line(center.v0 - offset, center.v1,
			center.v0 + offset, center.v1);
	      line.TransformTo(gframe);
	      AddLine(line);
	    }
	    {
	      Line line(center.v0, center.v1 - offset,
			center.v0, center.v1 + offset);
	      line.TransformTo(gframe);
	      AddLine(line);
	    }
	  }
	  else {
	    try { // west
	      if (travmap.grid.at(ix - 1, iy) < obstacle) {
		Line line(center.v0 - offset, center.v1 - offset,
			  center.v0 - offset, center.v1 + offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	    try { // east
	      if (travmap.grid.at(ix + 1, iy) < obstacle) {
		Line line(center.v0 + offset, center.v1 - offset,
			  center.v0 + offset, center.v1 + offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	    try { // south
	      if (travmap.grid.at(ix, iy - 1) < obstacle) {
		Line line(center.v0 - offset, center.v1 - offset,
			  center.v0 + offset, center.v1 - offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	    try { // north
	      if (travmap.grid.at(ix, iy + 1) < obstacle) {
		Line line(center.v0 - offset, center.v1 + offset,
			  center.v0 + offset, center.v1 + offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	  } // if (simple) ... else
	}
    
    // lower left
    GridFrame::position_t corner(gframe.LocalPoint(xbegin, ybegin));
    corner.v0 -= offset;
    corner.v1 -= offset;
    gframe.To(corner.v0, corner.v1);
    if( ! m_bbox)
      m_bbox.reset(new BBox(corner.v0, corner.v1, corner.v0, corner.v1));
    else
      m_bbox->Update(corner.v0, corner.v1);
    
    // upper left
    corner = gframe.LocalPoint(xbegin, yend - 1);
    corner.v0 -= offset;
    corner.v1 += offset;
    gframe.To(corner.v0, corner.v1);
    m_bbox->Update(corner.v0, corner.v1);
    
    // lower right
    corner = gframe.LocalPoint(xend - 1, ybegin);
    corner.v0 += offset;
    corner.v1 -= offset;
    gframe.To(corner.v0, corner.v1);
    m_bbox->Update(corner.v0, corner.v1);
    
    // upper right
    corner = gframe.LocalPoint(xend - 1, yend - 1);
    corner.v0 += offset;
    corner.v1 += offset;
    gframe.To(corner.v0, corner.v1);
    m_bbox->Update(corner.v0, corner.v1);
  }
  
  
  void World::
  AddKeyListener(boost::shared_ptr<KeyListener> listener) const
  {
    m_listener.push_back(listener);
  }
  
  
  void World::
  DispatchKey(unsigned char key) const
  {
    for(size_t il(0); il < m_listener.size(); ++il)
      m_listener[il]->KeyPressed(key);
  }
  
}
