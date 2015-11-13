/* Nepumuk Mpbile Robot Simulator v2
 *
 * Copyright (C) 2004 Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen
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


#include "View.hpp"
#include "Camera.hpp"
#include "Viewport.hpp"
#include "BBox.hpp"
#include "Drawing.hpp"
#include <boost/bind.hpp>


namespace npm2 {
  
  
  View::registry_t View::registry;
  
  
  View::
  View (const string & name)
    : fpplib::Configurable (name),
      camera_ (0)
  {
    registry.add (name, this);
    reflectCallback <string> ("camera", true, boost::bind (&View::setCamera, this, _1));
    reflectCallback <string> ("drawings", true, boost::bind (&View::addDrawing, this, _1));
    ////    reflectCallback <qhwin_s>("window", false, boost::bind(&View::setWindow, this, _1));
    reflectCallback <int> ("border", false, boost::bind (&View::setBorder, this, _1));
    reflectCallback <bool> ("squish", false, boost::bind (&View::setSquish, this, _1));
  }
  
  
  bool View::
  setCamera (const string & name)
  {
    Camera * cc (Camera::registry.find (name));
    if( !cc) {
      cerr << "ERROR in npm::View::setCamera: camera " << name << " not found\n"
	   << "  available Cameras:\n";
      for (Camera::registry_t::map_t::const_iterator ic (Camera::registry.map_.begin());
	   ic != Camera::registry.map_.end(); ++ic) {
	cerr << "    " << ic->first << ": " << ic->second->comment << "\n";
      }
      return false;
    }
    camera_ = cc;
    return true;
  }
  
  
  bool View::
  addDrawing (const string & name)
  {
    Drawing * dd (Drawing::registry.find (name));
    if (0 == dd) {
      cerr << "ERROR in npm::View::addDrawing: drawing " << name << " not found\n"
	   << "  available Drawings:\n";
      for (Drawing::registry_t::map_t::const_iterator id (Drawing::registry.map_.begin());
	   id != Drawing::registry.map_.end(); ++id) {
	cerr << "    " << id->first << ": " << id->second->comment << "\n";
      }
      return false;
    }
    drawings_.push_back (dd);
    return true;
  }
  
  
  bool View::
  draw ()
  {
    if ( ! camera_) {
      cerr << "ERROR in npm::View::draw: no Camera in View " << name << "\n";
      return false;
    }
    if (drawings_.empty()) {
      cerr << "ERROR in npm::View::draw: no Drawings in View " << name << "\n";
      return false;
    }
    
    camera_->configureView (*this);
    viewport_.pushOrtho ();
    for (size_t id (0); id < drawings_.size(); ++id) {
      drawings_[id]->draw();
    }
    viewport_.pop ();
    
    return true;
  }
  
  
  bool View::
  setWindow (double x0, double y0, double x1, double y1)
  {
    viewport_.updateSubwin (x0, y0, x1, y1);
    return true;
  }
  
  
  bool View::
  setBorder (int border)
  {
    viewport_.setBorder (border);    
    return true;
  }
  
  
  bool View::
  setSquish (bool squish)
  {
    viewport_.squish (squish);
    return true;
  }
  
  
  void View::
  setBounds (const BBox & bbox, double margin)
  {
    viewport_.updateBounds (bbox.x0() - margin, bbox.y0() - margin,
			    bbox.x1() + margin, bbox.y1() + margin);
  }
  
  
  void View::
  setBounds (double x0, double y0, double x1, double y1)
  {
    viewport_.updateBounds (x0, y0, x1, y1);
  }
  
  
  void View::
  reshape (int width, int height)
  {
    viewport_.updateShape (width, height);
  }
  
}
