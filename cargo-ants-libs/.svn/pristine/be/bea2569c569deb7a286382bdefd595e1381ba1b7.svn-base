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


#include "GridFrame.hpp"
#include <sfl/util/numeric.hpp>
#include <cmath>
#include <iostream>
#include <limits>


namespace sfl {


  GridFrame::
  GridFrame(double delta)
    : Frame(),
      m_delta(delta),
      m_delta_inv(1 / delta)
  {
  }


  GridFrame::
  GridFrame(double x, double y, double theta, double delta)
    : Frame(x, y, theta),
      m_delta(delta),
      m_delta_inv(1 / delta)
  {
  }


  GridFrame::
  GridFrame(const GridFrame & orig)
    : Frame(orig),
      m_delta(orig.m_delta),
      m_delta_inv(orig.m_delta_inv)
  {
  }


  GridFrame::
  GridFrame(const Frame & frame, double delta)
    : Frame(frame),
      m_delta(delta),
      m_delta_inv(1 / delta)
  {
  }


  void GridFrame::
  Configure(double position_x, double position_y, double position_theta,
	    double delta)
  {
    m_delta = delta;
    m_delta_inv = 1 / delta;
    Set(position_x, position_y, position_theta);
  }


  GridFrame::index_t GridFrame::
  GlobalIndex(double px, double py) const
  {
    From(px, py);
    return LocalIndex(px, py);
  }


  GridFrame::index_t GridFrame::
  GlobalIndex(position_t point) const
  {
    From(point.v0, point.v1);
    return LocalIndex(point);
  }


  GridFrame::index_t GridFrame::
  LocalIndex(double px, double py) const
  {
    return index_t(static_cast<int>(rint(px * m_delta_inv)),
		   static_cast<int>(rint(py * m_delta_inv)));
  }


  GridFrame::index_t GridFrame::
  LocalIndex(position_t point) const
  {
    return index_t(static_cast<int>(rint(point.v0 * m_delta_inv)),
		   static_cast<int>(rint(point.v1 * m_delta_inv)));
  }


  GridFrame::position_t GridFrame::
  GlobalPoint(ssize_t ix, ssize_t iy) const
  {
    position_t point(LocalPoint(ix, iy));
    To(point.v0, point.v1);
    return point;
  }


  GridFrame::position_t GridFrame::
  GlobalPoint(index_t index) const
  {
    position_t point(LocalPoint(index));
    To(point.v0, point.v1);
    return point;
  }


  GridFrame::position_t GridFrame::
  LocalPoint(ssize_t ix, ssize_t iy) const
  {
    return position_t(ix * m_delta, iy * m_delta);
  }


  GridFrame::position_t GridFrame::
  LocalPoint(index_t index) const
  {
    return position_t(index.v0 * m_delta, index.v1 * m_delta);
  }


  size_t GridFrame::
  DrawDDALine(ssize_t ix0, ssize_t iy0, ssize_t ix1, ssize_t iy1,
	      ssize_t xbegin, ssize_t xend,
	      ssize_t ybegin, ssize_t yend,
	      draw_callback & cb)
  {
    ssize_t ix(ix0);
    ssize_t iy(iy0);
    size_t count(0);
    if ((ix >= xbegin) && (iy >= ybegin) && (ix < xend) && (iy < yend)) {
      cb(ix, iy);
      ++count;
    }

    ssize_t dx(ix1 - ix0);
    ssize_t dy(iy1 - iy0);
    if(absval(dx) > absval(dy)){
      double slope(static_cast<double>(dy) / static_cast<double>(dx));
      if(dx < 0){
	slope = - slope;
	dx = -1;
      }
      else
	dx = 1;
      double yy(static_cast<double>(iy0));
      while(ix != ix1){
	ix += dx;
	yy += slope;
	iy = static_cast<ssize_t>(rint(yy));
	if ((ix >= xbegin) && (iy >= ybegin) && (ix < xend) && (iy < yend)) {
	  cb(ix, iy);
	  ++count;
	}
      }
      if(iy != iy1){
	if ((ix >= xbegin) && (iy >= ybegin) && (ix < xend) && (iy < yend)) {
	  cb(ix, iy1);
	  ++count;
	}
      }
    }
    else{
      double slope(static_cast<double>(dx) / static_cast<double>(dy));
      if(dy < 0){
	slope = - slope;
	dy = -1;
      }
      else
	dy = 1;
      double xx(static_cast<double>(ix0));
      while(iy != iy1){
	iy += dy;
	xx += slope;
	ix = static_cast<ssize_t>(rint(xx));
	if ((ix >= xbegin) && (iy >= ybegin) && (ix < xend) && (iy < yend)) {
	  cb(ix, iy);
	  ++count;
	}
      }
      if(ix != ix1){
	if ((ix >= xbegin) && (iy >= ybegin) && (ix < xend) && (iy < yend)) {
	  cb(ix1, iy);
	  ++count;
	}
      }
    }

    return count;
  }
  
  
  size_t GridFrame::
  DrawDDALine(ssize_t ix0, ssize_t iy0,
	      ssize_t ix1, ssize_t iy1,
	      draw_callback & cb)
  {
    return DrawDDALine(ix0, iy0, ix1, iy1,
		       std::numeric_limits<ssize_t>::min(),
		       std::numeric_limits<ssize_t>::max(), 
		       std::numeric_limits<ssize_t>::min(),
		       std::numeric_limits<ssize_t>::max(),
		       cb);
  }
  
  
  size_t GridFrame::
  DrawLocalLine(double x0, double y0, double x1, double y1,
		ssize_t xbegin, ssize_t xend,
		ssize_t ybegin, ssize_t yend,
		draw_callback & cb) const
  {
    return DrawDDALine(static_cast<ssize_t>(rint(x0 / m_delta)),
		       static_cast<ssize_t>(rint(y0 / m_delta)),
		       static_cast<ssize_t>(rint(x1 / m_delta)),
		       static_cast<ssize_t>(rint(y1 / m_delta)),
		       xbegin, xend, ybegin, yend, cb);
  }
  
  
  size_t GridFrame::
  DrawGlobalLine(double x0, double y0, double x1, double y1,
		 ssize_t xbegin, ssize_t xend,
		 ssize_t ybegin, ssize_t yend,
		 draw_callback & cb) const
  {
    From(x0, y0);
    From(x1, y1);
    return
      DrawLocalLine(x0, y0, x1, y1, xbegin, xend, ybegin, yend, cb);
  }
  
  
  static void draw_circle_points(ssize_t icx, ssize_t icy,
				 ssize_t ix, ssize_t iy,
				 GridFrame::draw_callback & cb)
  {
    if (ix == 0) {
      cb(icx     , icy + iy);
      cb(icx     , icy - iy);
      cb(icx + iy, icy     );
      cb(icx - iy, icy     );
    }
    else if (ix == iy) {
      cb(icx + ix, icy + iy);
      cb(icx - ix, icy + iy);
      cb(icx + ix, icy - iy);
      cb(icx - ix, icy - iy);
    }
    else if (ix < iy) {
      cb(icx + ix, icy + iy);
      cb(icx - ix, icy + iy);
      cb(icx + ix, icy - iy);
      cb(icx - ix, icy - iy);
      cb(icx + iy, icy + ix);
      cb(icx - iy, icy + ix);
      cb(icx + iy, icy - ix);
      cb(icx - iy, icy - ix);
    }
  }
  
  
  void GridFrame::
  DrawMidpointCircle(ssize_t icx, ssize_t icy, size_t irad,
		     draw_callback & cb)
  {
    ssize_t ix(0);
    ssize_t iy(irad);
    ssize_t pp((5 - iy * 4) / 4);
    
    draw_circle_points(icx, icy, ix, iy, cb);
    
    while (ix < iy) {
      ++ix;
      if (pp < 0)
	pp += 2 * ix + 1;
      else {
	--iy;
	pp += 2 * (ix - iy) + 1;
      }
      draw_circle_points(icx, icy, ix, iy, cb);
    }
  }
  
  
  void GridFrame::
  DrawLocalCircle(double cx, double cy, double rad,
		  draw_callback & cb) const
  {
    DrawMidpointCircle(static_cast<ssize_t>(rint(cx / m_delta)),
		       static_cast<ssize_t>(rint(cy / m_delta)),
		       static_cast<size_t>(rint(rad / m_delta)),
		       cb);
  }
  
  
  void GridFrame::
  DrawGlobalCircle(double cx, double cy, double rad,
		   draw_callback & cb) const
  {
    From(cx, cy);
    DrawLocalCircle(cx, cy, rad, cb);
  }
  
  
  GridFrame::dbg_draw_callback::
  dbg_draw_callback(std::ostream & os)
    : m_os(os)
  {
  }


  void GridFrame::dbg_draw_callback::
  operator () (ssize_t ix, ssize_t iy)
  {
    m_os << " (" << ix << ", " << iy << ")";
  }
  
  
  GridFrame::rangecheck_draw_callback::
  rangecheck_draw_callback(GridFrame::draw_callback & _cb,
			   ssize_t _xbegin, ssize_t _xend,
			   ssize_t _ybegin, ssize_t _yend)
    : cb(_cb),
      xbegin(_xbegin),
      xend(_xend),
      ybegin(_ybegin),
      yend(_yend),
      count(0)
  {
  }
  
  
  void GridFrame::rangecheck_draw_callback::
  operator () (ssize_t ix, ssize_t iy)
  {
    if ((xbegin <= ix) && (ix < xend) && (ybegin <= iy) && (iy < yend)) {
      cb(ix, iy);
      ++count;
    }
  }
  
  
  GridFrame & GridFrame::
  operator = (GridFrame const & rhs)
  {
    if (&rhs != this) {
      Frame::Set(rhs);
      m_delta = rhs.m_delta;
      m_delta_inv = rhs.m_delta_inv;
    }
    return *this;
  }
  
}
