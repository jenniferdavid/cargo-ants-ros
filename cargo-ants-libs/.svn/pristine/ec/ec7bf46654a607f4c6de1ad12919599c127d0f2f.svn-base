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


#ifndef NPM_VIEW_HPP
#define NPM_VIEW_HPP

#include <fpplib/configurable.hpp>
#include <vector>
#include <string>


namespace npm {
  
  class Camera;
  class BBox;
  class Drawing;
  
  struct qhwin_s {
    double x0, y0, x1, y1;
  };
  
  /**
     \brief Subwindow.
     
     A View is a subwindow of the graphic output. It has grown out of
     the need to handle OpenGL projection matrix and viewport
     configuration.
     
     Each View defines a region of graphic
     output, and maintains a list of Drawing instances that should be
     drawn into that region. An associated Camera is used to set the
     bounding box of the View's contents. This bounding box is
     independent of the View's location in the main graphics window
     (which is defined during construction or using View::Redefine() or
     View::Configure()).
  */
  class View
    : public fpplib::Configurable
  {
  public:
    typedef fpplib::Registry<View> registry_t;
    static registry_t registry;
    
    typedef enum { N, NE, E, SE, S, SW, W, NW, CENTER } anchor_t;
    
    explicit View(const std::string & name);
    
    void Configure(/// x-coordinate of lower-left corner, range = 0 to 1
		   double x,
		   /// y-coordinate of lower-left corner, range = 0 to 1
		   double y,
		   /// width (along y-axis), range = 0 to 1
		   double width,
		   /// height (along x-axis), range = 0 to 1
		   double height,
		   /// number of pixels to be subtracted around the edges
		   int border = 0,
		   /// how to anchor the drawn scene
		   anchor_t anchor = CENTER,
		   /// use an aspect ratio of 1, or scale to fit along x and y
		   bool lock_aspect = true);
    
    bool SetCamera(const std::string & name);
    bool AddDrawing(const std::string & name);
    void SavePNG(const std::string & filename);
    void SavePNG();
    
    bool rfctDraw();

    bool SetWindow(qhwin_s const &win);
    bool SetBorder(int border);
    bool SetAnchorCB(std::string const &anchor);
    void SetAnchor(anchor_t anchor);
    
    /// Set the bounding box of what's to be drawn inside the View.
    void SetBounds(const BBox & bbox, double margin = 0);
    
    /// Set the bounding box of what's to be drawn inside the View.
    void SetBounds(double x0, double y0, double x1, double y1,
		   double margin = 0);
    
    /// Inform the View of a reshape event of the main graphics window.
    void Reshape(int width,	///< new window width (in pixels)
		 int height	///< new window height (in pixels)
		 );
    
    /// Prepare the OpenGL parameters such that subsequent drawing
    /// commands draw inside the View, with the bounding box correctly
    /// set.
    void PrepareProjection() const;
    
    double BaseWidth() const;
    double BaseHeight() const;
    double BaseX() const;
    double BaseY() const;
    
    void UnlockAspectRatio();
    void LockAspectRatio();
    
    bool HaveCamera() const;
    
  private:
    std::string pcamera;
    std::vector<std::string> pdrawing;

    Camera * camera;
    std::vector<Drawing *> drawing;
    
    double basewidth, baseheight, basex, basey;
    int totalwidth, totalheight;
    int winwidth, winheight, winx, winy;
    double viewwidth, viewheight, viewx, viewy;
    double xmin, xmax,  ymin,  ymax;
    bool left, right, above, below;
    int winborder, doublewinborder;
    bool lockAspectRatio;
    
    int savecount;
    
    double Winwidth() const;
    double Winheight() const;
    int    Viewx() const;
    int    Viewy() const;
    int    Viewwidth() const;
    int    Viewheight() const;
    
    void CalculateViewport();
    
    std::string anchor_string;
  };

}

namespace std {

  ostream & operator << (ostream &os, npm::qhwin_s const &rhs);

}

#endif // NPM_VIEW_HPP
