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


#include "Polygon.hpp"
#include <sfl/util/functors.hpp>
#include <cmath>
#include <iostream>


using boost::shared_ptr;
using namespace std;


namespace sfl {


  Polygon::
  Polygon()
  {
  }


  Polygon::
  Polygon(const Polygon & p)
  {
    for(pointlist_t::const_iterator i(p.m_point.begin());
	i != p.m_point.end();
	++i)
      AddPoint((*i)->X(), (*i)->Y());
  }


  void Polygon::
  AddPoint(double x,
	   double y)
  {
    m_point.push_back(shared_ptr<Point>(new Point(x, y)));
  }


  shared_ptr<Polygon> Polygon::
  CreateConvexHull()
    const
  {
    // constructs ccw convex hull
    // (ccw in the sense of the list of resulting points)
    
    shared_ptr<Polygon> hull(new Polygon());
    
    if(m_point.empty())
      return hull;
    
    // find lowest point in vector
    pointlist_t::const_iterator ipoint(m_point.begin());
    shared_ptr<const Point> lowest( * ipoint);
    ++ipoint;
    while(ipoint != m_point.end()){
      if(((*ipoint)->Y() < lowest->Y()) ||   // take lowest, or
	 (((*ipoint)->Y() == lowest->Y()) && // if tied,
	  ((*ipoint)->X() < lowest->X())))   // take leftmost point
	lowest = *ipoint;
      ++ipoint;
    }

    // perform Jarvis' march
    shared_ptr<const Point> p0(lowest);
    shared_ptr<const Point> pc;
    do{ // while(p0 != lowest);
      // add new p0 to polygon
      hull->AddPoint(p0->X(), p0->Y());
    
      // find Point pc that is clockwise from all others
      int count(0);
      for(ipoint = m_point.begin(); ipoint != m_point.end(); ++ipoint){
	// skip p0
	if(*ipoint == p0)
	  continue;
	
	// count how many points have been explored
	++count;
	
	// assume first point is clockwise
	if(count == 1){
	  pc = *ipoint;
	  continue;
	}
	
	// determine whether ipoint is clockwise from pc
	double z(((*ipoint)->X() - p0->X()) * (pc->Y() - p0->Y()) -
		 ((*ipoint)->Y() - p0->Y()) * (pc->X() - p0->X()));
	if((z > 0) ||    // ipoint is clockwise, or
	   ((z == 0) &&  // if tied, take farthest point
	    (sqrt(pow((*ipoint)->X() - p0->X(), 2) +
		  pow((*ipoint)->Y() - p0->Y(), 2))
	     >
	     sqrt(pow(    pc->X() - p0->X(), 2) +
		  pow(    pc->Y() - p0->Y(), 2)))))
	  pc = *ipoint;
      } // for(ipoint = point.begin(); ipoint != point.end(); ++ipoint){
      
      // prepare to find next point on hull
      p0 = pc;
      
      // until reaching the lowest point again
    } while(p0 != lowest);
    
    return hull;
  }
  

  void Polygon::
  ComputeBoundingBox(double & x0,
		     double & y0,
		     double & x1,
		     double & y1)
    const
  {
    if(m_point.empty()){
      x0 = 0;
      y0 = 0;
      x1 = 0;
      y1 = 0;
      return;
    }
  
    pointlist_t::const_iterator ipoint(m_point.begin());
    x0 = (*ipoint)->X();
    x1 = (*ipoint)->X();
    y0 = (*ipoint)->Y();
    y1 = (*ipoint)->Y();
    ++ipoint;
    while(ipoint != m_point.end()){
      if((*ipoint)->X() < x0)
	x0 = (*ipoint)->X();
      if((*ipoint)->X() > x1)
	x1 = (*ipoint)->X();
      if((*ipoint)->Y() < y0)
	y0 = (*ipoint)->Y();
      if((*ipoint)->Y() > y1)
	y1 = (*ipoint)->Y();
      ++ipoint;
    }
  }


  bool Polygon::
  Contains(double x,
	   double y)
    const
  {
    if(m_point.size() < 3)
      return false;
  
    pointlist_t::const_iterator ipoint(m_point.begin());
    const Point * previous(ipoint->get());
    ++ipoint;
    while(ipoint != m_point.end()){
      if(((*ipoint)->X() - x) * (previous->Y() - y) -
	 ((*ipoint)->Y() - y) * (previous->X() - x) > 0)
	return false;

      previous = ipoint->get();
      ++ipoint;
    }
    ipoint = m_point.begin();
    if(((*ipoint)->X() - x) * (previous->Y() - y) -
       ((*ipoint)->Y() - y) * (previous->X() - x) > 0)
      return false;

    return true;
  }


  shared_ptr<Polygon> Polygon::
  CreateGrownPolygon(double padding)
    const
  {
    shared_ptr<Polygon> polygon(new Polygon());
    
    if(m_point.size() < 3)
      return polygon;
    
    size_t lastpoint = m_point.size() - 1;
    
    // special treatment for first point
    Point v0(m_point[lastpoint]->X() - m_point[0]->X(),
	      m_point[lastpoint]->Y() - m_point[0]->Y());
    Point v1(m_point[1]->X() - m_point[0]->X(),
	      m_point[1]->Y() - m_point[0]->Y());
    double l0(sqrt(v0.X() * v0.X() + v0.Y() * v0.Y()));
    double l1(sqrt(v1.X() * v1.X() + v1.Y() * v1.Y()));

    Point u(v0.X() / l0 + v1.X() / l1,
	     v0.Y() / l0 + v1.Y() / l1);

    double mu(padding * l0 / (v0.X() * u.Y() - v0.Y() * u.X()));
    
    polygon->AddPoint(m_point[0]->X() + mu * u.X(),
		      m_point[0]->Y() + mu * u.Y());
  
    // normal treatment for all except last point
    for(size_t ii(1); ii < lastpoint; ++ii){
      v0.X(m_point[ii - 1]->X() - m_point[ii]->X());
      v0.Y(m_point[ii - 1]->Y() - m_point[ii]->Y());
      v1.X(m_point[ii + 1]->X() - m_point[ii]->X());
      v1.Y(m_point[ii + 1]->Y() - m_point[ii]->Y());
      l0 = sqrt(v0.X() * v0.X() + v0.Y() * v0.Y());
      l1 = sqrt(v1.X() * v1.X() + v1.Y() * v1.Y());
    
      u.X(v0.X() / l0 + v1.X() / l1);
      u.Y(v0.Y() / l0 + v1.Y() / l1);
    
      mu = padding * l0 / (v0.X() * u.Y() - v0.Y() * u.X());
    
      polygon->AddPoint(m_point[ii]->X() + mu * u.X(),
			m_point[ii]->Y() + mu * u.Y());
    }

    // special treatment for last point
    v0.X(m_point[lastpoint - 1]->X() - m_point[lastpoint]->X());
    v0.Y(m_point[lastpoint - 1]->Y() - m_point[lastpoint]->Y());
    v1.X(m_point[0]->X() - m_point[lastpoint]->X());
    v1.Y(m_point[0]->Y() - m_point[lastpoint]->Y());
    l0 = sqrt(v0.X() * v0.X() + v0.Y() * v0.Y());
    l1 = sqrt(v1.X() * v1.X() + v1.Y() * v1.Y());
  
    u.X(v0.X() / l0 + v1.X() / l1);
    u.Y(v0.Y() / l0 + v1.Y() / l1);
  
    mu = padding * l0 / (v0.X() * u.Y() - v0.Y() * u.X());
  
    polygon->AddPoint(m_point[lastpoint]->X() + mu * u.X(),
		      m_point[lastpoint]->Y() + mu * u.Y());
  
    // finished
    return polygon;
  }


  double Polygon::
  CalculateRadius()
    const
  {
    if(m_point.empty())
      return 0;

    double r2Max(0);
    for(pointlist_t::const_iterator ipoint(m_point.begin());
	ipoint != m_point.end();
	++ipoint){
      double r2((*ipoint)->X() * (*ipoint)->X() +
		(*ipoint)->Y() * (*ipoint)->Y());
      if(r2 > r2Max)
	r2Max = r2;
    }

    return sqrt(r2Max);
  }
  
  
  shared_ptr<Line> Polygon::
  _GetLine(size_t index)
    const
  {
    if(index >= m_point.size())
      return shared_ptr<Line>();
    if(index == m_point.size() - 1)
      return shared_ptr<Line>(new Line(m_point[m_point.size() - 1]->X(),
				       m_point[m_point.size() - 1]->Y(),
				       m_point[0]->X(),
				       m_point[0]->Y()));
    return shared_ptr<Line>(new Line(m_point[index    ]->X(),
				     m_point[index    ]->Y(),
				     m_point[index + 1]->X(),
				     m_point[index + 1]->Y()));
  }
  
  
  ostream & operator << (ostream & os,
			 const Polygon & polygon)
  {
    Polygon::pointlist_t::const_iterator ipoint(polygon.m_point.begin());
    os << "(";
    while(ipoint != polygon.m_point.end()){
      os << "(" << (*ipoint)->X() << ", " << (*ipoint)->Y() << ")";
      ++ipoint;
    }
    os << ")";
    return os;
  }
  
  
  const Point * Polygon::
  GetPoint(size_t index)
    const
  {
    if(index >= m_point.size())
      return 0;
    return m_point[index].get();
  }

}
