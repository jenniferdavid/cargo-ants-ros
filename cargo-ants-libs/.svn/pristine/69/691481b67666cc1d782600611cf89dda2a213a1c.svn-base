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


#include "BBox.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/util/Point.hpp>
#include <sfl/util/Line.hpp>


using namespace sfl;


namespace npm {
  
  
  BBox::
  BBox(double x0, double y0, double x1, double y1)
    : m_x0(minval(x0, x1)),
      m_y0(minval(y0, y1)),
      m_x1(maxval(x0, x1)),
      m_y1(maxval(y0, y1))
  {
  }
  
  
  BBox::
  BBox(const Point & p0, const Point & p1)
    : m_x0(minval(p0.X(), p1.X())),
      m_y0(minval(p0.Y(), p1.Y())),
      m_x1(maxval(p0.X(), p1.X())),
      m_y1(maxval(p0.Y(), p1.Y()))
  {
  }
  
  
  BBox::
  BBox(const sfl::Line & line)
    : m_x0(minval(line.X0(), line.X1())),
      m_y0(minval(line.Y0(), line.Y1())),
      m_x1(maxval(line.X0(), line.X1())),
      m_y1(maxval(line.Y0(), line.Y1()))
  {
  }
  
  
  BBox::
  BBox(const BBox & original)
    : m_x0(original.m_x0),
      m_y0(original.m_y0),
      m_x1(original.m_x1),
      m_y1(original.m_y1)
  {
  }
  
  
  void BBox::
  Update(double x, double y)
  {
    m_x0 = minval(m_x0, x);
    m_y0 = minval(m_y0, y);
    m_x1 = maxval(m_x1, x);
    m_y1 = maxval(m_y1, y);
  }
  
  
  void BBox::
  Update(const Point & point)
  {
    Update(point.X(), point.Y());
  }
  
  
  void BBox::
  Update(const Line & line)
  {
    Update(line.X0(), line.Y0());
    Update(line.X1(), line.Y1());
  }
  
  
  void BBox::
  Update(const BBox & bbox)
  {
    Update(bbox.m_x0, bbox.m_y0);
    Update(bbox.m_x1, bbox.m_y1);    
  }

}
