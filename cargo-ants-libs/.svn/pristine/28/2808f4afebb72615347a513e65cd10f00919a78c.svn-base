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


#include "Object.hpp"
#include "Sensor.hpp"
#include "BBox.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/util/Line.hpp>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  Object::
  Object(const string & name_, const std::string & comment)
    : name(name_)
      ////rfct: do we need comment?
  {
  }
  
  
  Object::
  Object(const Object & original)
    : name(original.name),
      ////rfct: do we need comment?
      m_radius(original.m_radius)
  {
    for(size_t il(0); il < original.m_local.size(); ++il)
      m_local.push_back(shared_ptr<Line>(new Line(*original.m_local[il])));
    for(size_t il(0); il < original.m_global.size(); ++il)
      m_global.push_back(shared_ptr<Line>(new Line(*original.m_global[il])));
    if(original.m_lazy_transform)
      m_lazy_transform.reset(new Frame(*original.m_lazy_transform));
    if(original.m_local_bb)
      m_local_bb.reset(new BBox(*original.m_local_bb));
    if(original.m_global_bb)
      m_global_bb.reset(new BBox(*original.m_global_bb));
  }
  
  
  void Object::
  AddLine(const Line & line)
  {
    m_local.push_back(shared_ptr<Line>(new Line(line)));
    if( ! m_local_bb)
      m_local_bb.reset(new BBox(line));
    m_local_bb->Update(line);
    const double rad(sqrt(maxval(sqr(line.X0()) + sqr(line.Y0()),
				 sqr(line.X1()) + sqr(line.Y1()))));
    if(rad > m_radius)
      m_radius = rad;
  }
  
  
  void Object::
  TransformTo(const Frame & frame)
  {
    m_lazy_transform.reset(new Frame(frame));
  }
  
  
  void Object::
  LazyTransform() const
  {
    if(m_local.empty())
      return;
    if( ! m_lazy_transform){
      if(m_local.size() != m_global.size()){
	m_global_bb.reset(new BBox(*m_local[0]));
	for(size_t il(0); il < m_local.size(); ++il){
	  if(m_global.size() <= il)
	    m_global.push_back(shared_ptr<Line>(new Line(*m_local[il])));
	  else
	    *m_global[il] = *m_local[il];
	  m_global_bb->Update(*m_global[il]);
	}
      }
      return;
    }
    if(m_global.empty())
      m_global.push_back(shared_ptr<Line>(new Line(*m_local[0])));
    else
      *m_global[0] = *m_local[0];
    m_global[0]->TransformTo(*m_lazy_transform);
    m_global_bb.reset(new BBox(*m_global[0]));
    for(size_t il(1); il < m_local.size(); ++il){
      if(m_global.size() <= il)
	m_global.push_back(shared_ptr<Line>(new Line(*m_local[il])));
      else
	*m_global[il] = *m_local[il];
      m_global[il]->TransformTo(*m_lazy_transform);
      m_global_bb->Update(*m_global[il]);
    }
    m_lazy_transform.reset();
  }
  
  
  void Object::
  UpdateSensor(Sensor & sensor) const
  {
    LazyTransform();
    for(size_t il(0); il < m_global.size(); ++il)
      sensor.StepUpdate(*m_global[il]);
  }
  
  
  shared_ptr<const BBox> Object::
  GetGlobalBB() const
  {
    LazyTransform();
    return m_global_bb;
  }
  
  
  shared_ptr<const Line> Object::
  GetLocalLine(size_t index) const
  {
    if(m_local.size() <= index)
      return shared_ptr<Line>();
    return m_local[index];
  }
  
  
  shared_ptr<const Line> Object::
  GetGlobalLine(size_t index) const
  {
    if(m_local.empty() || (m_local.size() <= index))
      return shared_ptr<Line>();
    LazyTransform();
    return m_global[index];
  }
  
}
