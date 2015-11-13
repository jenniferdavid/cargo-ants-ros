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


#include "Bubble.hpp"
#include <sfl/util/numeric.hpp>
#include <cmath>


using boost::shared_ptr;
using std::pair;


namespace sfl {


  const double Bubble::DEFAULTALPHAINT(0.1);
  const double Bubble::DEFAULTALPHAEXT(0.5);


  void Bubble::
  Construct()
  {
    _previous            =  0;
    _next                =  0;
    _radius              = -1;
    _radius2             = -1;
    _dprevious           = -1;
    _dnext               = -1;
    _path_length         = -1;
    _min_ignore_distance =  0;
    _fext                = pair<double, double>(0, 0);
    _fint                = pair<double, double>(0, 0);
    _alpha_int           = DEFAULTALPHAINT;
    _alpha_ext           = DEFAULTALPHAEXT;
  }


  void Bubble::
  Configure(double cutoff_distance,
	    pair<double, double> position)
  {
    Construct();
    _cutoff_distance = cutoff_distance;
    _position = position;
  }


  void Bubble::
  CopyConstruct(const Bubble & original)
  {
    _previous             = 0;
    _next                 = 0;
    _position             = original._position;
    _closest              = original._closest;
    _radius               = original._radius;
    _radius2              = original._radius2;
    _dprevious            = original._dprevious;
    _dnext                = original._dnext;
    _path_length          = original._path_length;
    _cutoff_distance      = original._cutoff_distance;
    _ignore_distance      = original._ignore_distance;
    _ignore_distance2     = original._ignore_distance2;
    _min_ignore_distance  = original._min_ignore_distance;
    _fext                 = original._fext;
    _fint                 = original._fint;
    _alpha_int            = original._alpha_int;
    _alpha_ext            = original._alpha_ext;
  }


  void Bubble::
  SetMinIgnoreDistance(double d)
  {
    _min_ignore_distance = d;

    if(_ignore_distance < _min_ignore_distance)
      SetIgnoreDistance(_min_ignore_distance);
  }


  void Bubble::
  SetIgnoreDistance(double d)
  {
    if(d < _min_ignore_distance)
      return;

    _ignore_distance = d;
    _ignore_distance2 = d * d;
  }


  void Bubble::
  UpdateExternalParameters(shared_ptr<const Scan> scan,
			   double ignore_radius)
  {
    _radius2 = -1;

    int count(0);

    const size_t nscans(scan->data.size());
    for(size_t is(0); is < nscans; ++is)
      if(scan->data[is].in_range && (scan->data[is].rho >= ignore_radius)){
	const scan_data & gdata(scan->data[is]);
	double dx(gdata.globx - _position.first);
	double dy(gdata.globy - _position.second);
	double r2(dx * dx + dy * dy);
	if( (r2 < _radius2) || (_radius2 < 0) ){
	  count++;
	  _closest.first = gdata.globx;
	  _closest.second = gdata.globy;
	  _radius2 = r2;
	}
      }
    
    if(_radius2 <= 0){
      _radius = -1;
      _fext = pair<double, double>(0, 0);
      return;
    }
    
    if(_radius2 < _ignore_distance2)
      _radius2 = _ignore_distance2;

    _radius = sqrt(_radius2);
    
    if((_radius >= _cutoff_distance) || (_radius < epsilon)){
      _fext = pair<double, double>(0, 0);
      return;
    }
    
    _fext.first =
      - (_cutoff_distance - _radius)
      * (_closest.first - _position.first)
      / _radius;
    
    _fext.second =
      - (_cutoff_distance - _radius)
      * (_closest.second - _position.second)
      / _radius;
  }



  void Bubble::
  ResetInternalParameters()
  {
    _dprevious = -1;
    _dnext = -1;
    _path_length = -1;

    _fint = pair<double, double>(0, 0);
  }



  void Bubble::
  UpdateInternalParameters()
  {

    ///////////////////////////////////////////////
    // problem if _dnext very small but _dprevious not
    // would still give f=(0, 0) even though it should
    // probably be calculated according to _dprevious
    // (and vice-versa)
    //////////////////////////////////////////////

    if((_dprevious < 0) && (_previous != 0)){
      double dx(_previous->_position.first - _position.first);
      double dy(_previous->_position.second - _position.second);
      _dprevious = sqrt(dx * dx + dy * dy);

      double fx, fy;
      if(_dprevious >= epsilon){
	fx = dx / _dprevious;
	fy = dy / _dprevious;
      }
      else{
	fx = 0;
	fy = 0;
      }

      _fint.first += fx;
      _fint.second += fy;

      _previous->_dnext = _dprevious;
      _previous->_fint.first -= fx;
      _previous->_fint.second -= fy;
    }

    if((_dnext < 0) && (_next != 0)){
      double dx(_next->_position.first - _position.first);
      double dy(_next->_position.second - _position.second);
      _dnext = sqrt(dx * dx + dy * dy);

      double fx, fy;
      if(_dnext >= epsilon){
	fx = dx / _dnext;
	fy = dy / _dnext;
      }
      else{
	fx = 0;
	fy = 0;
      }

      _fint.first += fx;
      _fint.second += fy;

      _next->_dprevious = _dnext;
      _next->_fint.first -= fx;
      _next->_fint.second -= fy;
    }
  }



  void Bubble::
  ApplyForces()
  {
    double alpha(0);

    if(_cutoff_distance > epsilon)
      alpha = minval(1.0, _radius / _cutoff_distance);

    _position.first +=
      alpha * (_alpha_int * _fint.first + _alpha_ext * _fext.first);

    _position.second +=
      alpha * (_alpha_int * _fint.second + _alpha_ext * _fext.second);
  }



  bool Bubble::
  CheckOverlap(const Bubble & bubble1,
	       const Bubble & bubble2,
	       double min_normal)
  {
    double dx, dy, distance, radial, normal;
    int overlapType;

    return InformativeCheckOverlap(bubble1, bubble2, min_normal,
				   dx, dy, distance, radial, normal,
				   overlapType);
  }



  bool Bubble::
  InformativeCheckOverlap(const Bubble & bubble1,
			  const Bubble & bubble2,
			  double min_normal,
			  double & dx,
			  double & dy,
			  double & distance,
			  double & radial,
			  double & normal,
			  int & overlap_type)
  {
    overlap_type = InformativeOverlap(bubble1, bubble2,
				      dx, dy, distance, radial, normal);
    
    if(overlap_type == NOOVERLAP)
      return false;
    
    if((overlap_type == PARTIALOVERLAP) && (normal < min_normal) &&
       (radial > 0) && (radial <= distance))
      return false;
    
    return true;
  }
  


  Bubble::overlap_t Bubble::
  InformativeOverlap(const Bubble & bubble1,
		     const Bubble & bubble2,
		     double & dx,
		     double & dy,
		     double & distance,
		     double & radial,
		     double & normal)
  {
    dx = bubble2._position.first - bubble1._position.first;
    dy = bubble2._position.second - bubble1._position.second;
    double d2(dx * dx + dy * dy);
    distance = sqrt(d2);

    if(distance > bubble1._radius + bubble2._radius)
      return NOOVERLAP;

    if(distance < absval(bubble2._radius - bubble1._radius))
      return FULLOVERLAP;

    if(distance < epsilon)
      return SAMECENTER;

    radial = 0.5 * (bubble1._radius * bubble1._radius -
		    bubble2._radius * bubble2._radius +
		    d2 ) / distance;
    normal = 2 * sqrt(bubble1._radius * bubble1._radius - radial * radial);

    return PARTIALOVERLAP;
  }



  double Bubble::
  Distance(const Bubble & bubble1,
	   const Bubble & bubble2)
  {
    double dx(bubble1._position.first - bubble2._position.first);
    double dy(bubble1._position.second - bubble2._position.second);

    return sqrt(dx * dx + dy * dy);
  }



}
