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


#include "numeric.hpp"
#include <cmath>


namespace sfl {


  int QuadraticEquation(double a,
			double b,
			double c,
			double & x1,
			double & x2)
  {
  if(absval(a) < epsilon){
    if(absval(b) < epsilon)
      return 0;

    x1 = - c / b;
    return 1;
  }
  
  if(absval(b) < epsilon){
    double sq(- c / a);
    if(sq < 0)
      return 0;

    if(absval(sq) < epsilon){
      x1 = 0;
      return 1;
    }

    x1 = sqrt(sq);
    x2 = - x1;
    return 2;
  }

  if(absval(c) < epsilon){
    x1 = 0;
    x2 = - b / a;
    return 2;
  }

  double root(b * b - 4 * a * c);
  if(root < 0)
    return 0;
  
  if(absval(root) < epsilon * epsilon){
    x1 = - 0.5 * b / a;
    return 1;
  }
  
  root = sqrt(root);
  x1 = - 0.5 * (b + root) / a;
  x2 = - 0.5 * (b - root) / a;
  return 2;
  }
  
  
  int LineCircleIntersect(double x0,
			  double y0,
			  double x1,
			  double y1,
			  double cx,
			  double cy,
			  double cr,
			  double &q1x,
			  double &q1y,
			  double &q2x,
			  double &q2y,
			  bool &valid1,
			  bool &valid2)
  {
    const double delta_cx(cx - x0);
    const double delta_cy(cy - y0);
    const double delta_px(x1 - x0);
    const double delta_py(y1 - y0);

    const double a(sqr(delta_px) + sqr(delta_py));
    const double c(sqr(delta_cx) + sqr(delta_cy) - sqr(cr));
    const double b(- 2 * (delta_cx * delta_px + delta_cy * delta_py));
  
    double l1, l2;
    switch(QuadraticEquation(a, b, c, l1, l2)){
    case 0:
      valid1 = false;
      valid2 = false;
      return 0;
    case 1:
      q1x = x0 + l1 * delta_px;
      q1y = y0 + l1 * delta_py;
      valid1 = (l1 >= 0) && (l1 <= 1);
      valid2 = false;
      return valid1 ? 1 : 0;
    }
    
    q1x = x0 + l1 * delta_px;
    q1y = y0 + l1 * delta_py;
    q2x = x0 + l2 * delta_px;
    q2y = y0 + l2 * delta_py;
    valid1 = (l1 >= 0) && (l1 <= 1);
    valid2 = (l2 >= 0) && (l2 <= 1);
    return valid1 ? (valid2 ? 2 : 1) : (valid2 ? 1 : 0);
  }
  
  
  double LineRayIntersect(double p0x, double p0y, double p1x, double p1y,
			  double rx, double ry, double dx, double dy)
  {
    // describe line as {point, vector} in global frame
    const double v0x(p1x - p0x);
    const double v0y(p1y - p0y);
    
    // determinant of intersection matrix
    const double det(dx * v0y - v0x * dy);
    if(absval(det) <= epsilon)
      return -1;	      // parallel, or zero length or direction
    
    // look where the ray intersects with the line segment
    const double mu0((dx * (ry - p0y) - dy * (rx - p0x)) / det);
    if((mu0 < 0) || (mu0 > 1))
      return -1;		// outside of segment
    
    // calculate distance from ray origin to intersection
    const double mu1((v0x * (ry - p0y) - v0y * (rx - p0x)) / det);
    if(mu1 < 0)
      return -1;		// reverse intersection
    
    return mu1;
  }
  
}
