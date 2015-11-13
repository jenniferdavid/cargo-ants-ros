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

#ifndef NPM2_DRAWING_HPP
#define NPM2_DRAWING_HPP

#include <fpplib/configurable.hpp>


namespace npm2 {

  using namespace std;
  
  
  /**
     Abstract drawing functionality and interface. Drawings are used
     to define what's shown in each View. More than one Drawing can be
     registered for a View.
  */
  class Drawing
    : public fpplib::Configurable
  {
  public:
    typedef fpplib::Registry <Drawing, false> registry_t;
    static registry_t registry;
    
    const string comment;
    
    Drawing (const string & name,
	     const string & comment);
    
    virtual ~Drawing ();
    
    /**
       The whole idea of this class is to have subclasses define this
       method. When invoked, it should draw whatever is to be drawn.
    */
    virtual void draw () = 0;
  };

}

#endif // NPM_DRAWING_HPP
