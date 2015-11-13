/* 
 * Copyright (C) 2005
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


#include "Hull.hpp"
#include "functors.hpp"


using boost::shared_ptr;
using namespace std;


namespace sfl {


  Hull::
  Hull():
    m_npoints(0)
  {
  }
  
  
  Hull::
  Hull(const Hull & original):
    m_npoints(original.m_npoints)
  {
    for(subhulls_t::const_iterator is(original.m_subhulls.begin());
	is != original.m_subhulls.end();
	++is)
      m_subhulls.push_back(shared_ptr<Polygon>(new Polygon( ** is)));
  }
  
  
  Hull::
  Hull(const Polygon & poly)
    : m_npoints(0)
  {
    AddPolygon(poly);
  }
  
  
  void Hull::
  AddPolygon(const Polygon & polygon)
  {
    m_subhulls.push_back(polygon.CreateConvexHull());
    m_npoints += m_subhulls.back()->GetNPoints();
  }
  
  
  shared_ptr<Hull> Hull::
  CreateGrownHull(double amount)
    const
  {
    shared_ptr<Hull> grown(new Hull());
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is){
      grown->m_subhulls.push_back((*is)->CreateGrownPolygon(amount));
      grown->m_npoints += grown->m_subhulls.back()->GetNPoints();
    }
    return grown;
  }
  
  
  double Hull::
  CalculateRadius()
    const
  {
    double radius(-1);
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is){
      const double r((*is)->CalculateRadius());
      if((radius < 0) || (r > radius))
	radius = r;
    }
    return radius;
  }
  
  
  shared_ptr<Line> Hull::
  _GetLine(size_t index)
    const
  {
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is){
      const size_t npoints((*is)->GetNPoints());
      if(index < npoints)
	return (*is)->_GetLine(index);
      index -= npoints;
    }
    return shared_ptr<Line>();
  }
  
  
  bool Hull::
  Contains(double x,
	   double y)
    const
  {
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is)
      if((*is)->Contains(x, y))
	return true;
    return false;
  }
  
  
  shared_ptr<const Polygon> Hull::
  GetPolygon(size_t index)
    const
  {
    if(index >= m_subhulls.size())
      return shared_ptr<const Polygon>();
    return m_subhulls[index];
  }
  
  
  HullIterator::
  HullIterator(const Hull & hull)
    : m_hull(hull),
      m_valid(false)
  {
    // find first valid pair of points
    for(m_ipoly = 0; m_ipoly < m_hull.GetNPolygons(); ++m_ipoly){
      m_poly = m_hull.GetPolygon(m_ipoly);
      if(m_poly->GetNPoints() <= 1)
	continue;
      m_p0 = m_poly->GetPoint(m_poly->GetNPoints() - 1);
      m_p1 = m_poly->GetPoint(0);
      m_ipoint = 0;
      m_valid = true;
      break;
    }
  }
  
  
  void HullIterator::
  Increment()
  {
    if( ! m_valid)
      return;;
    
    ++m_ipoint;
    if(m_ipoint < m_poly->GetNPoints()){
      m_p0 = m_p1;
      m_p1 = m_poly->GetPoint(m_ipoint);
      return;
    }
    
    ++m_ipoly;
    if(m_ipoly >= m_hull.GetNPolygons()){
      m_valid = false;
      return;
    }
    
    m_poly = m_hull.GetPolygon(m_ipoly);
    while(m_poly->GetNPoints() <= 1){
      ++m_ipoly;
      if(m_ipoly >= m_hull.GetNPolygons()){
	m_valid = false;
	return;
      }
      m_poly = m_hull.GetPolygon(m_ipoly);      
    }
    m_p0 = m_poly->GetPoint(m_poly->GetNPoints() - 1);
    m_p1 = m_poly->GetPoint(0);
    m_ipoint = 0;
  }
  
}
