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


#ifndef NPM_BBOX_HPP
#define NPM_BBOX_HPP


namespace sfl {
  class Point;
  class Line;
}


namespace npm {
  
  
  class BBox
  {
  public:
    BBox(double x0, double y0, double x1, double y1);
    BBox(const sfl::Point & p0, const sfl::Point & p1);
    explicit BBox(const sfl::Line & line);
    BBox(const BBox & original);
    
    void Update(double x, double y);
    void Update(const sfl::Point & point);
    void Update(const sfl::Line & line);
    void Update(const BBox & bbox);
    
    double X0() const { return m_x0; }
    double Y0() const { return m_y0; }
    double X1() const { return m_x1; }
    double Y1() const { return m_y1; }
    
  protected:
    double m_x0, m_y0, m_x1, m_y1;
  };

}

#endif // NPM_BBOX_HPP
