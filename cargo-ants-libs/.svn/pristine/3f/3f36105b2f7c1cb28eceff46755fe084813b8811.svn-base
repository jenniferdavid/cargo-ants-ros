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


#ifndef NPM_CAMERA_HPP
#define NPM_CAMERA_HPP

#include <fpplib/registry.hpp>


namespace npm {

  class View;
  
  /**
     Define region to be drawn in a View.
     
     Cameras are stored (by name) in the CameraManager and provide a
     generic way to set a View to a specific bounding box. 
  */
  class Camera
  {
  public:
    typedef fpplib::Registry <Camera, false> registry_t;
    static registry_t registry;
    
    const std::string name;
    const std::string comment;
    
    /**
       Camera instances always need a Manager. You can typically just
       use the singleton instance by passing in
       'Instance<UniqueManager<Camera> >()'.
       
       \note Well, you can pass in a smart null pointer to avoid that
       if you must. Automatically registering can be problematic if
       you use proxies e.g. for lazy initialization, because all
       cameras have to be registered before the layout config file is
       parsed.
    */
    Camera(const std::string & name,
	   const std::string & comment);
    
    virtual ~Camera() {}
    
    /**
       Configure a View to the Camera's bounding box.
       
       Each View has one Camera, and before drawing into it, the
       Simulator calls this method on it's registered Camera, providing
       a way of implementing Views that follow an object or with
       adjustable zoom.
    */
    virtual void ConfigureView(View & view) = 0;
  };

}

#endif // NPM_CAMERA_HPP
