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


#ifndef NPM_DRAWING_HPP
#define NPM_DRAWING_HPP

#include <fpplib/registry.hpp>


namespace npm {
  
  
  /**
     \brief Abstract drawing functionality and interface.
     
     Drawings are used to define what's shown in each View. More than
     one Drawing can be registered for a View. The DrawingManager
     handles Drawing instances by name. In order to draw something,
     write a subclass of Drawing, implement the Drawing::Draw() method,
     and register instances with the DrawingManager. Inside of your
     Draw() method, you can conveniently use the wrapper methods for
     coding what should be drawn.
  */
  class Drawing
  {
  public:
    typedef fpplib::Registry <Drawing, false> registry_t;
    static registry_t registry;
    
    const std::string name;
    const std::string comment;
    
    /**
       Drawing instances always need a Manager. You can typically just
       use the singleton instance by passing in
       'Instance<UniqueManager<Drawing> >()' which is declared in
       Manager.hpp.
    */
    Drawing(const std::string & name,
	    const std::string & comment);
    
    virtual ~Drawing() {}
    
    /**
       The whole idea of this class is to have subclasses define this
       method. When invoked, it should draw whatever is to be drawn.
    */
    virtual void Draw() = 0;
  };

}

#endif // NPM_DRAWING_HPP
