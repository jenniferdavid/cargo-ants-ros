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


#include "Goal.hpp"
#include <sfl/util/numeric.hpp>
#include <iostream>


namespace sfl {
  
  
  const double Goal::DEFAULTGOALDR = 0.25;
  const double Goal::DEFAULTGOALDTHETA = M_PI;


  Goal::
  Goal():
    _x(0),
    _y(0),
    _theta(0),
    _dr(DEFAULTGOALDR),
    _dr2(DEFAULTGOALDR * DEFAULTGOALDR),
    _dtheta(DEFAULTGOALDTHETA),
    _isVia(false)
  {
  }


  Goal::
  Goal(const Goal & original):
    _x(original._x),
    _y(original._y),
    _theta(original._theta),
    _dr(original._dr),
    _dr2(original._dr2),
    _dtheta(original._dtheta),
    _isVia(original._isVia)
  {
  }


  Goal::
  Goal(double x,
       double y,
       double theta,
       double dr,
       double dtheta,
       bool viaGoal):
    _x(x),
    _y(y),
    _theta(theta),
    _dr(absval(dr)),
    _dr2(dr * dr),
    _dtheta(absval(dtheta)),
    _isVia(viaGoal)
  {
  }


  void Goal::
  Set(const Goal & goal)
  {
    _x      = goal._x;
    _y      = goal._y;
    _theta  = goal._theta;
    _dr     = goal._dr;
    _dr2    = goal._dr2;
    _dtheta = goal._dtheta;
    _isVia  = goal._isVia;
  }


  void Goal::
  Set(double x,
      double y,
      double theta,
      double dr,
      double dtheta,
      bool viaGoal)
  {
    _x      = x;
    _y      = y;
    _theta  = theta;
    _dr     = dr;
    _dr2    = sqr(dr);
    _dtheta = dtheta;
    _isVia  = viaGoal;
  }


  double Goal::
  X()
    const
  {
    return _x;
  }


  double Goal::
  Y()
    const
  {
    return _y;
  }


  double Goal::
  Theta()
    const
  {
    return _theta;
  }


  double Goal::
  Dr()
    const
  {
    return _dr;
  }


  double Goal::
  Dr2()
    const
  {
    return _dr2;
  }


  double Goal::
  Dtheta()
    const
  {
    return _dtheta;
  }


  bool Goal::
  IsVia()
    const
  {
    return _isVia;
  }


  bool Goal::
  DistanceReached(const Frame & position)
    const
  {
    double dx(_x - position.X());
    double dy(_y - position.Y());

    if((dx*dx + dy*dy) > _dr2)
      return false;

    return true;
  }


  bool Goal::
  HeadingReached(const Frame & position,
		 bool goForward,
		 double & dheading)
    const
  {
    if(_dtheta >= M_PI){
      dheading = 0;
      return true;
    }
  
    dheading = _theta - position.Theta();
    if( ! goForward)
      dheading = M_PI - dheading;
    dheading = mod2pi(dheading);
  
    if(absval(dheading) > _dtheta)
      return false;
  
    return true;
  }


  bool Goal::
  Reached(const Frame & position,
	  bool goForward)
    const
  {
    if(_isVia){
      if(DistanceReached(position)){
	return true;
      }
      return false;
    }

    double dheading;
    if(DistanceReached(position) &&
       HeadingReached(position, goForward, dheading)){
      return true;
    }
    return false;
  }
  
  
  std::ostream & operator << (std::ostream & os, const Goal & goal)
  {
    return os << "(" << goal._x << ", " << goal._y << ", " << goal._theta
	      << " / " << goal._dr << ", " << goal._dtheta << ")";
  }
  
}
