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


#ifndef SUNFLOWER_NUMERIC_HPP
#define SUNFLOWER_NUMERIC_HPP


#include <cmath>

#ifndef M_PI
// the usual Windows issue...
# define M_PI 3.14159265358979323846
#endif // M_PI


namespace sfl {


  static const double epsilon = 1e-9;


  /**
     Evaluates to the minimum of it's two arguments.

     \note Would have liked to use const references instead of values,
     but that creates problems for comparing against constants
     (e.g. things defined as "static const double"). This doesn't really
     matter for builtin-types, but comparing more complex types entails
     constructor and destructor calls...
  */
  template<typename T>
  T minval(T a, T b)
  {
    return a < b ? a : b;
  }


  /**
     Evaluates to the maximum of it's two arguments.
  */
  template<typename T>
  T maxval(T a, T b)
  {
    return a > b ? a : b;
  }
  
  
  /**
     Evaluates to the middle argument or one of the bounds if the
     value is too great or too small.
  */
  template<typename T>
  T boundval(T lower_bound, T value, T upper_bound)
  {
    return maxval(minval(value, upper_bound), lower_bound);
  }
  
  
  /**
     Evaluates to the absolute value of it's argument.
  */
  template<typename T>
  T absval(const T & a)
  {
    return a > 0 ? T(a) : T(-a);
  }


  /**
     Calculates the square of its argument.
  */
  template<typename T>
  T sqr(T x)
  {
    return x * x;
  }


  /**
     Set an angle to the range -pi < angle <= +pi
  */
  inline double mod2pi(double x)
  {
    x = fmod(x, 2 * M_PI);
    if(x > M_PI)
      x -= 2 * M_PI;
    else if(x <= - M_PI)
      x += 2 * M_PI;
    
    return x;
  }


  /**
     Solves the quadratic equation "a * x ^ 2 + b * x + c == 0".

     \note Handles special cases "gracefully" based on the properties
     (absval(a) < epsilon), (absval(b) < epsilon), (absval(c) <
     epsilon), and (absval(b*b-4*a*c) < epsilon*epsilon).

     \return The number of solutions (0, 1, or 2), it only sets x1 / x2
     for the returned number of solutions (e.g. if it returns 1, only x1
     contains a useful number).
  */
  int QuadraticEquation(double a, double b, double c,
			double & x1, double & x2);
  
  
  /**
     Intersect a line (x0, y0, x1, y1) with a circle (cx, cy) of radius cr.
     
     \return The number of intersections (0, 1, or 2).
     
     \note If the function returns 1, you still have to check which of
     the two solutions is valid, don't just blindly use the first one!
  */
  int LineCircleIntersect(/** x-coordinate of the first line endpoint */
			  double x0,
			  /** y-coordinate of the first line endpoint */
			  double y0,
			  /** x-coordinate of the second line endpoint */
			  double x1,
			  /** y-coordinate of the second line endpoint */
			  double y1,
			  /** x-coordinate of circle's center */
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
			  bool & valid2);

  /**
     Intersect a Line (finite segment) and a Ray (half-infinite).
     
     \return
     - -1 if no intersection exists
     - mu >= 0 if an intersection exists, where mu is the factor
       needed to multiply the Ray's directing vector in order to
       reach the intersection point from the Ray's base point. In
       other words, if the Ray's directing vector is a unit
       vector, mu is the distance that needs to be travelled
       along the Ray before hitting the Line.
  */
  double LineRayIntersect(/** x-coordinate of the first line endpoint */
			  double x0,
			  /** y-coordinate of the first line endpoint */
			  double y0,
			  /** x-coordinate of the second line endpoint */
			  double x1,
			  /** y-coordinate of the second line endpoint */
			  double y1,
			  /** x-coordinate of ray origin */
			  double rx,
			  /** y-coordinate of ray origin */
			  double ry,
			  /** x-component of ray direction vector */
			  double dx,
			  /** y-component of ray direction vector */
			  double dy);
  
}

#endif // SUNFLOWER_NUMERIC_HPP
