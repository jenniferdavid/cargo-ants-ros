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


#include "Line.hpp"
#include <sfl/util/numeric.hpp>
#include <iostream>


namespace sfl {


  Line::
  Line():
    p0(0.0, 0.0),
    p1(0.0, 0.0)
  {
  }


  Line::
  Line(const Point & _p0,
       const Point & _p1):
    p0(_p0),
    p1(_p1)
  {
  }


  Line::
  Line(double x0,
       double y0,
       double x1,
       double y1):
    p0(x0, y0),
    p1(x1, y1)
  {
  }


  void Line::
  TransformTo(const Frame & t)
  {
    p0.TransformTo(t);
    p1.TransformTo(t);
  }


  void Line::
  TransformFrom(const Frame & t)
  {
    p0.TransformFrom(t);
    p1.TransformFrom(t);
  }


  void Line::
  CircleIntersect(double cx,
		  double cy,
		  double cr,
		  double &q1x,
		  double &q1y,
		  double &q2x,
		  double &q2y,
		  bool &valid1,
		  bool &valid2)
    const
  {
    LineCircleIntersect(p0.X(), p0.Y(), p1.X(), p1.Y(),
			cx, cy, cr, q1x, q1y, q2x, q2y, valid1, valid2);
  }
  

  Line & Line::
  operator = (const Line & orig)
  {
    p0 = orig.p0;
    p1 = orig.p1;
    return * this;
  }
  
}

namespace std {
  
  ostream & operator << (ostream & os, sfl::Line const &rhs)
  {
    return os << "("
	      << rhs.X0() << ", " << rhs.Y0() << ", "
	      << rhs.X1() << ", " << rhs.Y1() << ")";
  }
  
}
