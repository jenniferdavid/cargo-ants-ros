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


#include "View.hpp"
#include "PNGImage.hpp"
#include "Camera.hpp"
#include <npm/BBox.hpp>
#include <npm/pdebug.hpp>
#include "Drawing.hpp"
#include "wrap_glu.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/util/numeric.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <stdio.h>


using namespace sfl;
using namespace std;


namespace npm {
  
  
  View::registry_t View::registry;
  
  
  View::
  View(const std::string & name)
    : fpplib::Configurable(name),
      camera(0),
      savecount(0)
  {
    registry.add(name, this);
    Configure(0, 0, 1, 1);
    reflectParameter("camera", &pcamera);
    reflectVectorParameter("drawings", &pdrawing);
    reflectCallback<qhwin_s>("window", false, boost::bind(&View::SetWindow, this, _1));
    reflectCallback<int>("border", false, boost::bind(&View::SetBorder, this, _1));
    reflectCallback<string>("anchor", false, boost::bind(&View::SetAnchorCB, this, _1));
  }
  
  
  void View::
  Configure(double x,
	    double y,
	    double width,
	    double height,
	    int border,
	    anchor_t anchor,
	    bool lock_aspect)
  {
    winborder = border;
    doublewinborder = 2 * border;
    lockAspectRatio = lock_aspect;
  
    basewidth  = maxval(0.0, minval(1.0, width));
    baseheight = maxval(0.0, minval(1.0, height));
    basex      = maxval(0.0, minval(1.0, x));
    basey      = maxval(0.0, minval(1.0, y));
  
    SetAnchor(anchor);		// calls CalculateViewport() at the end
  }


  bool View::
  SetCamera(const string &name)
  {
    Camera *cc(Camera::registry.find(name));
    if( !cc) {
      return false;
    }
    camera = cc;
    return true;
  }
  
  
  bool View::
  AddDrawing(const string &name)
  {
    PVDEBUG ("DBG View::AddDrawing(`%s')\n", name.c_str());
    Drawing * dd(Drawing::registry.find(name));
    if(0 == dd) {
      cerr << "ERROR in npm::View::AddDrawing: drawing " << name << " not found\n"
	   << "  available Drawings:\n";
      for (Drawing::registry_t::map_t::const_iterator id(Drawing::registry.map_.begin());
	   id != Drawing::registry.map_.end(); ++id) {
	cerr << "    " << id->first << ": " << id->second->comment << "\n";
      }
      return false;
    }
    drawing.push_back(dd);
    return true;
  }


  bool View::
  rfctDraw()
  {
    if (camera == 0) {
      if ( !SetCamera(pcamera)) {
	return false;
      }
    }
    
    if (drawing.empty()) {
      if (pdrawing.empty()) {
	cerr << "ERROR in npm::View::Draw: no drawings were specified\n";
	return false;
      }
      for (size_t id(0); id < pdrawing.size(); ++id) {
	if ( !AddDrawing(pdrawing[id])) {
	  return false;
	}
      }
    }
    
    camera->ConfigureView(*this);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    PrepareProjection();
    for_each(drawing.begin(), drawing.end(), mem_fun(&Drawing::Draw));
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    
    return true;
  }


  void View::
  SavePNG(const string &filename)
  {
#ifndef NPM_HAVE_PNG
    std::cerr << __FILE__": " << __func__ << ": PNG not supported in this build\n";
#else
    PNGImage image(Viewwidth(), Viewheight());
    image.read_framebuf(Viewx(), Viewy());
    image.write_png(filename);
#endif
  }


  void View::
  SavePNG()
  {
    char countbuf[6];
    if(savecount >= 1000000)	// hax: hardcoded cycle limit
      savecount = 0;
    sprintf(countbuf, "%05d", savecount++);
    SavePNG(string("pics/") + name + to_string(countbuf) + ".png");
  }


  bool View::
  SetWindow(qhwin_s const &win)
  {
    basewidth  = maxval(0.0, minval(1.0, win.x1 - win.x0));
    baseheight = maxval(0.0, minval(1.0, win.y1 - win.y0));
    basex      = maxval(0.0, minval(1.0, win.x0));
    basey      = maxval(0.0, minval(1.0, win.y0));

    Reshape(totalwidth, totalheight);
    
    return true;
  }


  bool View::
  SetBorder(int border)
  {
    winborder = border;
    doublewinborder = 2 * border;

    CalculateViewport();
    
    return true;
  }
  
  
  bool View::
  SetAnchorCB(string const &anchor)
  {
    static map<string, anchor_t> mm;
    if (mm.empty()) {
      mm["N"] = N;
      mm["NE"] = NE;
      mm["E"] = E;
      mm["SE"] = SE;
      mm["S"] = S;
      mm["SW"] = SW;
      mm["W"] = W;
      mm["NW"] = NW;
      mm["CENTER"] = CENTER;
      mm["n"] = N;
      mm["ne"] = NE;
      mm["e"] = E;
      mm["se"] = SE;
      mm["s"] = S;
      mm["sw"] = SW;
      mm["w"] = W;
      mm["nw"] = NW;
      mm["center"] = CENTER;
    }
    map<string, anchor_t>::const_iterator im(mm.find(anchor));
    if (im == mm.end()) {
      return false;
    }
    SetAnchor(im->second);
    return true;
  }
  
  
  void View::
  SetAnchor(anchor_t anchor)
  {
    switch(anchor){
    case N:
      anchor_string = "N";
      left = true;
      right = true;
      above = false;
      below = true;
      break;
    case NE:
      anchor_string = "NE";
      left = true;
      right = false;
      above = false;
      below = true;
      break;
    case E:
      anchor_string = "E";
      left = true;
      right = false;
      above = true;
      below = true;
      break;
    case SE:
      anchor_string = "SE";
      left = true;
      right = false;
      above = true;
      below = false;
      break;
    case S:
      anchor_string = "S";
      left = true;
      right = true;
      above = true;
      below = false;
      break;
    case SW:
      anchor_string = "SW";
      left = false;
      right = true;
      above = true;
      below = false;
      break;
    case W:
      anchor_string = "W";
      left = false;
      right = true;
      above = true;
      below = true;
      break;
    case NW:
      anchor_string = "NW";
      left = false;
      right = true;
      above = false;
      below = true;
      break;
    case CENTER:
    default:
      anchor_string = "CENTER";
      left = true;
      right = true;
      above = true;
      below = true;
    }

    CalculateViewport();
  }


  void View::
  SetBounds(const class BBox &bbox,
	    double margin)
  {
    xmin = bbox.X0() - margin;
    xmax = bbox.X1() + margin;
    ymin = bbox.Y0() - margin;
    ymax = bbox.Y1() + margin;
    
    CalculateViewport();
  }


  void View::
  SetBounds(double x0, double y0, double x1, double y1, double margin)
  {
    xmin = x0 - margin;
    xmax = x1 + margin;
    ymin = y0 - margin;
    ymax = y1 + margin;
    
    CalculateViewport();
  }


  void View::
  Reshape(int width,
	  int height)
  {
    totalwidth = width;
    totalheight = height;

    winwidth  = (int) floor(basewidth * (double) width);
    winheight = (int) floor(baseheight * (double) height);
    winx      = (int) ceil(basex * (double) width);
    winy      = (int) ceil(basey * (double) height);

    CalculateViewport();
  }


  void View::
  PrepareProjection()
    const
  {
    glLoadIdentity();
    gluOrtho2D(xmin, xmax, ymin, ymax);
    glViewport(Viewx(), Viewy(), Viewwidth(), Viewheight());
  }


  double View::
  BaseWidth()
    const
  {
    return basewidth;
  }


  double View::
  BaseHeight()
    const
  {
    return baseheight;
  }


  double View::
  BaseX()
    const
  {
    return basex;
  }


  double View::
  BaseY()
    const
  {
    return basey;
  }


  void View::
  UnlockAspectRatio()
  {
    lockAspectRatio = false;
    CalculateViewport();
  }


  void View::
  LockAspectRatio()
  {
    lockAspectRatio = true;
    CalculateViewport();
  }


  double View::
  Winwidth()
    const
  {
    return winwidth - doublewinborder;
  }


  double View::
  Winheight()
    const
  {
    return winheight - doublewinborder;
  }


  int View::
  Viewx()
    const
  {
    return winborder + winx + static_cast<int>(floor(viewx));
  }


  int View::
  Viewy()
    const
  {
    return winborder + winy + static_cast<int>(floor(viewy));
  }


  int View::
  Viewwidth()
    const
  {
    return static_cast<int>(ceil(viewwidth));
  }


  int View::
  Viewheight()
    const
  {
    return static_cast<int>(ceil(viewheight));
  }


  void View::
  CalculateViewport()
  {
    if(lockAspectRatio){
      double winratio(Winwidth() / Winheight());
      double w(xmax - xmin);
      double h(ymax - ymin);
      double viewratio(w / h);
    
      if(viewratio < winratio){	// put some space left and right
	double epsilon(Winheight() / h);
	viewwidth = w * epsilon;
	viewheight = Winheight();
	if(left && right)
	  viewx = 0.5 * (Winwidth() - viewwidth);
	else if(left)
	  viewx = (Winwidth() - viewwidth);
	else
	  viewx = 0;
	viewy = 0;
      }
      else{				// put some space above and below
	double epsilon(Winwidth() / w);
	viewwidth = Winwidth();
	viewheight = h * epsilon;
	viewx = 0;
	if(above && below)
	  viewy = 0.5 * (Winheight() - viewheight);
	else if(below)
	  viewy = (Winheight() - viewheight);
	else
	  viewy = 0;
      }
    }
    else{
      viewwidth = Winwidth();
      viewheight = Winheight();
      viewx = 0;
      viewy = 0;
    }
  }
  
  
  bool View::
  HaveCamera() const
  {
    return camera != 0;
  }
  
}

namespace std {

  ostream & operator << (ostream &os, npm::qhwin_s const &rhs)
  {
    return os << "(" << rhs.x0 << "," << rhs.y0 << "," << rhs.x1 << "," << rhs.y1 << ")";
  }
  
}
