/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#ifndef NPM2_BBOX_HPP
#define NPM2_BBOX_HPP


namespace sfl {
  class Point;
  class Line;
}


namespace npm2 {
  
  using namespace sfl;
  
  
  class BBox
  {
  public:
    BBox ();
    BBox (double x0, double y0, double x1, double y1);
    BBox (Point const & p0, Point const & p1);
    explicit BBox (Line const & line);
    
    void reset ();
    bool isValid () const { return x0_ <= x1_; }
    
    void update (double xx, double yy);
    void update (Point const & point);
    void update (Line const & line);
    void update (BBox const & bbox);
    
    double x0 () const { return x0_; }
    double y0 () const { return y0_; }
    double x1 () const { return x1_; }
    double y1 () const { return y1_; }
    
  protected:
    double x0_, y0_, x1_, y1_;
  };

}

#endif // NPM2_BBOX_HPP
