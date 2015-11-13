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


#ifndef SUNFLOWER_POINT_HPP
#define SUNFLOWER_POINT_HPP


#include <sfl/util/Frame.hpp>


namespace sfl {

  /**
     2D point without covariance information.
  */
  class Point
  {
  public:
    /**
       Construct the point (x, y).
    */
    Point(double x, double y);

    /**
       \return The point's x-coordinate.
    */
    inline double X() const;

    /**
       \return The point's y-coordinate.
    */
    inline double Y() const;

    /**
       Sets the point's x-coordinate.
    */
    inline void X(double x);

    /**
       Sets the point's y-coordinate.
    */
    inline void Y(double y);

    /**
       Transforms the point to the given coordinate frame. Calls
       Frame::To() for (x, y).
    */    
    void TransformTo(const Frame & t);

    /**
       Transforms the point from the given coordinate frame. Calls
       Frame::From() for (x, y).
    */    
    void TransformFrom(const Frame & t);
    
    Point & operator = (const Point & orig);
    
    double _x, _y;
  };
  

  double Point::
  X()
    const
  {
    return _x;
  }


  double Point::
  Y()
    const
  {
    return _y;
  }


  void Point::
  X(double x)
  {
    _x = x;
  }


  void Point::
  Y(double y)
  {
    _y = y;
  }

}

#endif // SUNFLOWER_POINT_HPP
