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


#ifndef SUNFLOWER_LINE_HPP
#define SUNFLOWER_LINE_HPP


#include <sfl/util/Point.hpp>
#include <sfl/util/Frame.hpp>


namespace sfl {

  /**
     Line segment without covariance.
  */
  class Line
  {
  public:
    /**
       Default constructor (all members are zero).
    */
    Line();
    
    /**
       Construct a line given its two endpoints.
    */
    Line(const Point & p0, const Point & p1);

    /**
       Construct a line given its two endpoints.
    */
    Line(double x0, double y0, double x1, double y1); 
  
    /**
       \return The first endpoint.
    */
    inline const Point & P0() const;

    /**
       \return The second endpoint.
    */
    inline const Point & P1() const;

    /**
       \return The first endpoint's x-coordinate.
    */
    inline double X0() const;

    /**
       \return The first endpoint's y-coordinate.
    */
    inline double Y0() const;

    /**
       \return The second endpoint's x-coordinate.
    */
    inline double X1() const;

    /**
       \return The second endpoint's y-coordinate.
    */
    inline double Y1() const;

    /**
       Transforms the line to the given coordinate frame. Calls
       Frame::To() for each endpoint.
    */
    void TransformTo(const Frame & t);

    /**
       Transforms the line from the given coordinate frame. Calls
       Frame::From() for each endpoint.
    */
    void TransformFrom(const Frame & t);

    /**
       Calculates the intersection(s) between a circle and the Line.
       
       \note The implementation has been moved to numeric.cpp, this
       method simply calls LineCircleIntersect().
    */
    void CircleIntersect(/** x-coordinate of circle's center */
			 double cx,
			 /** y-coordinate of circle's center */
			 double cy,
			 /** radius of the circle */
			 double cr,
			 /** (return) x-coordinate of first intersection */
			 double & q1x,
			 /** (return) y-coordinate of first intersection */
			 double & q1y,
			 /** (return) x-coordinate of second intersection */
			 double & q2x,
			 /** (return) y-coordinate of second intersection */
			 double & q2y,
			 /** (return) is (q1x, q1y) a valid intersection? */
			 bool & valid1,
			 /** (return) is (q2x, q2y) a valid intersection? */
			 bool & valid2
			 ) const;
    
    Line & operator = (const Line & orig);
    
    Point p0, p1;
  };


  const Point & Line::
  P0()
    const
  {
    return p0;
  }


  const Point & Line::
  P1()
    const
  {
    return p1;
  }


  double Line::
  X0()
    const
  {
    return p0.X();
  }


  double Line::
  Y0()
    const
  {
    return p0.Y();
  }


  double Line::
  X1()
    const
  {
    return p1.X();
  }


  double Line::
  Y1()
    const
  {
    return p1.Y();
  }

}


namespace std {
  
  ostream & operator << (ostream & os, sfl::Line const &rhs);
  
}

#endif // SUNFLOWER_LINE_HPP
