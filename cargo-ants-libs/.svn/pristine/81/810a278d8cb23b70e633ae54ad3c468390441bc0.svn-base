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

#ifndef NPM2_BODY_HPP
#define NPM2_BODY_HPP

#include <npm2/BBox.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/util/Line.hpp>
#include <vector>


namespace npm2 {
  
  using namespace sfl;
  using namespace std;
  
  
  class Body
  {
  public:
    typedef vector <Line> lines_t;
    
    /** Adds a line wrt the local reference frame. */
    void addLine (Line const & line);
    
    /** Adds a line wrt the local reference frame. */
    void addLine (double x0, double y0, double x1, double y1);
    
    /** Transforms the lines to the given global reference frame. */
    void transformTo (Frame const & global);
    
    /** Returns lines wrt the global reference frame. */
    lines_t const & getLines () const { return global_lines_; }
    
    /** Returns the bounding box of the lines wrt the global reference frame. */
    BBox const & getBBox () const { return bbox_; }
    
  protected:
    lines_t local_lines_;
    lines_t global_lines_;
    BBox bbox_;			// global
  };
  
}

#endif // NPM2_BODY_HPP
