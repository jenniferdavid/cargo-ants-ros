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

#ifndef NPM2_VIEW_HPP
#define NPM2_VIEW_HPP

#include <npm2/Viewport.hpp>
#include <fpplib/configurable.hpp>


namespace npm2 {

  using namespace std;
  
  class Camera;
  class BBox;
  class Drawing;
  
  /**
     A View is a subwindow of the graphic output. It has a Camera
     which defines the region of the plane shown in the view, a list
     of Drawings to display, and uses a Viewport to handle OpenGL
     projection and viewport configuration.
  */
  class View
    : public fpplib::Configurable
  {
  public:
    typedef fpplib::Registry<View> registry_t;
    static registry_t registry;
    
    explicit View (const string & name);
    
    bool setCamera (const string & name);
    bool addDrawing (const string & name);
    bool setWindow (double x0, double y0, double x1, double y1);
    bool setBorder (int border);
    bool setSquish (bool squish);
    
    /**
       Set the bounding box of what's to be drawn inside the
       View. Intended to be used by the Camera registered for this
       View.
    */
    void setBounds (const BBox & bbox, double margin);
    
    void setBounds (double x0, double y0, double x1, double y1);
    
    /**
       Inform the View of a reshape event of the main graphics window.
    */
    void reshape (int width,	///< new window width (in pixels)
		  int height	///< new window height (in pixels)
		  );
    
    bool draw();
    
  private:
    Viewport viewport_;
    Camera * camera_;
    vector <Drawing*> drawings_;
  };
  
}

#endif // NPM_VIEW_HPP
