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


#include "Point.hpp"


namespace sfl {
  
  
  Point::
  Point(double x, double y):
    _x(x),
    _y(y)
  {
  }


  void Point::
  TransformTo(const Frame & t)
  {
    t.To(_x, _y);
  }


  void Point::
  TransformFrom(const Frame & t)
  {
    t.From(_x, _y);
  }


  Point & Point::
  operator = (const Point & orig)
  {
    _x = orig._x;
    _y = orig._y;
    return * this;
  }
  
}
