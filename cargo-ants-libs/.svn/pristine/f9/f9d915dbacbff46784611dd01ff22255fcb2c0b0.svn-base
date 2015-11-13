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


namespace npm2 {
  
  
  BBox::
  BBox ()
  {
    reset();
  }
  
  
  BBox::
  BBox (double x0, double y0, double x1, double y1)
    : x0_ (minval (x0, x1)),
      y0_ (minval (y0, y1)),
      x1_ (maxval (x0, x1)),
      y1_ (maxval (y0, y1))
  {
  }
  
  
  BBox::
  BBox (Point const & p0, const Point & p1)
    : x0_ (minval (p0.X(), p1.X())),
      y0_ (minval (p0.Y(), p1.Y())),
      x1_ (maxval (p0.X(), p1.X())),
      y1_ (maxval (p0.Y(), p1.Y()))
  {
  }
  
  
  BBox::
  BBox (Line const & line)
    : x0_ (minval (line.X0(), line.X1())),
      y0_ (minval (line.Y0(), line.Y1())),
      x1_ (maxval (line.X0(), line.X1())),
      y1_ (maxval (line.Y0(), line.Y1()))
  {
  }
  
  
  void BBox::
  reset ()
  {
    x0_ = 1.0;
    y0_ = 0.0;
    x1_ = -1.0;
    y1_ = 0.0;
  }
  
  
  void BBox::
  update (double xx, double yy)
  {
    if (isValid()) {
      x0_ = minval (x0_, xx);
      y0_ = minval (y0_, yy);
      x1_ = maxval (x1_, xx);
      y1_ = maxval (y1_, yy);
    }
    else {
      x0_ = xx;
      y0_ = yy;
      x1_ = xx;
      y1_ = yy;
    }
  }
  
  
  void BBox::
  update (Point const & point)
  {
    update (point.X(), point.Y());
  }
  
  
  void BBox::
  update (Line const & line)
  {
    update (line.X0(), line.Y0());
    update (line.X1(), line.Y1());
  }
  
  
  void BBox::
  update (const BBox & bbox)
  {
    if (bbox.isValid()) {
      update (bbox.x0_, bbox.y0_);
      update (bbox.x1_, bbox.y1_);
    }
  }

}
