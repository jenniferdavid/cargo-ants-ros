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

#ifndef NPM2_VIEWPORT_HPP
#define NPM2_VIEWPORT_HPP

namespace npm2 {
    
    
  class Viewport
  {
  public:
    Viewport ();
    Viewport (double subwin_x0, double subwin_y0, double subwin_x1, double subwin_y1);
      
    /** The subwindow, in relative coordinates (0...1). */
    void updateSubwin (double x0, double y0, double x1, double y1);
    void updateShape (int width, int height);
    void updateBounds (double x0, double y0, double x1, double y1);
    void pushOrtho ();
    void pop ();
    
    int setBorder (int border);
    bool squish (bool enable);
    
  protected:
    void updatePadding ();
      
    struct {
      double x0, y0, width, height;
    } subwin_;
      
    struct {
      double x0, x1, y0, y1, cx, cy;
    } bounds_;
      
    struct {
      int x0, y0, width, height, winwidth, winheight;
    } shape_;
      
    struct {
      int x0, y0, width, height;
    } padding_;
    
    int border_;
    bool squish_;
    bool dirty_;
  };

}

#endif // NPM2_VIEWPORT_HPP
