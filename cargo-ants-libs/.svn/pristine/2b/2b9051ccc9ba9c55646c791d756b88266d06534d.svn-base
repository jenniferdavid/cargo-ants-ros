/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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
#include <boost/bind.hpp>


namespace npm2 {
  
  
  Object::registry_t Object::registry;
  
  
  Object::
  Object (string const & name)
    : fpplib::Configurable (name),
      parent_ (0)
  {
    registry.add (name, this);
    reflectParameter ("mount", &mount_);
    reflectCallback<string> ("parent", true, boost::bind (&Object::initParent, this, _1));
    reflectCallback<Line> ("lines", true, boost::bind (&Object::addLine, this, _1));
  }
  
  
  Object::
  ~Object()
  {
    registry.remove (name, this);
  }
  
  
  void Object::
  setParent (Object * parent)
  {
    if (parent_) {
      parent_->children_.erase (this);
    }
    parent_ = parent;
    if (parent_) {
      parent_->children_.insert (this);
    }
  }
  
  
  bool Object::
  initParent (string const & name)
  {
    Object * parent (registry.find(name));
    if( ! parent) {
      return false;
    }
    if (parent == this) {
      return false;
    }
    setParent (parent);
    return true;
  }
  
  
  Object * Object::
  attach (Object * new_parent)
  {
    if ( ! parent_) {
      setParent (new_parent);
      return 0;
    }
    
    Object * old_parent (parent_);
    setParent (new_parent);
    
    if (new_parent) {
      mount_ = global_;
      new_parent->getGlobal().From (mount_);
    }
    
    return old_parent;
  }
  
  
  bool Object::
  addLine (Line const & line)
  {
    body_.addLine (line);
    return true;
  }
  
  
  void Object::
  updateTransform ()
  {
    global_ = motion_;
    mount_.To (global_);
    if (parent_) {
      parent_->global_.To (global_);
    }
    body_.transformTo (global_);
    
    bbox_.reset();
    for (children_t::iterator ic (children_.begin()); ic != children_.end(); ++ic) {
      (*ic)->updateTransform();
      bbox_.update ((*ic)->bbox_);
    }
    bbox_.update (body_.getBBox());
  }
  
  
  void Object::
  updateSensor (Sensor * sensor) const
  {
    sensor->sensorUpdate (body_);
    for (children_t::iterator ic (children_.begin()); ic != children_.end(); ++ic) {
      (*ic)->updateSensor (sensor);
    }
  }
  
}
