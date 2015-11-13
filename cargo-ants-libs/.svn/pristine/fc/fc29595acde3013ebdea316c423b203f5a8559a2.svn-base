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


#ifndef NPM_OBJECT_HPP
#define NPM_OBJECT_HPP


#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>


namespace sfl {
  class Frame;
  class Line;
}


namespace npm {
  
  class Sensor;
  class BBox;
  
  /**
     Basic simulated entity.
  */
  class Object
  {
  public:
    const std::string name;
    ////rfct: const std::string comment;

    Object(const std::string & name, const std::string & comment);
    Object(const Object & original);
    
    void AddLine(const sfl::Line & line);
    void TransformTo(const sfl::Frame & frame);
    void UpdateSensor(Sensor & sensor) const;
    
    size_t GetNlines() const { return m_local.size(); }
    double GetRadius() const { return m_radius; }
    boost::shared_ptr<const BBox> GetLocalBB() const { return m_local_bb; }
    boost::shared_ptr<const BBox> GetGlobalBB() const;
    boost::shared_ptr<const sfl::Line> GetLocalLine(size_t index) const;
    boost::shared_ptr<const sfl::Line> GetGlobalLine(size_t index) const;
    
  protected:
    void LazyTransform() const;
    
    typedef std::vector<boost::shared_ptr<sfl::Line> > line_t;
    line_t m_local;
    mutable line_t m_global;
    mutable boost::shared_ptr<sfl::Frame> m_lazy_transform;
    boost::shared_ptr<BBox> m_local_bb;
    mutable boost::shared_ptr<BBox> m_global_bb;
    double m_radius;
  };
  
}

#endif // NPM_OBJECT_HPP
