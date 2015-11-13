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


#ifndef SUNFLOWER_POLYGON_HPP
#define SUNFLOWER_POLYGON_HPP


#include <sfl/util/Line.hpp>
#include <sfl/util/Point.hpp>
#include <sfl/util/numeric.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iosfwd>


namespace sfl {


  /**
     A simple polygon implementation that allows calculating the
     convex hull, and some more straightforward functions.
  */
  class Polygon
  {
  public:
    Polygon();
    Polygon(const Polygon & p);
    
    /** Adds the specified point to the polygon. */
    void AddPoint(double x, double y);
    
    /**
       Performs Jarvis' March on the polygon and thus constructs and
       returns the convex hull. Needed prior to using
       Polygon::Contains() and Polygon::CreateGrownPolygon().
    */
    boost::shared_ptr<Polygon> CreateConvexHull() const;
    
    /** Calculates the bounding box of the polygon. */
    void ComputeBoundingBox(double & x0, double & y0, double & x1, double & y1)
      const;
    
    /**
       \note Only correct for ccw convex hulls constructed by ConvexHull().
    */
    bool Contains(double x, double y) const;
    
    /**
       \note Only correct for ccw convex hulls constructed by
       ConvexHull().
       
       \todo The algorithm is a bit ad-hoc and could surely be
       optimized.
       
    */
    boost::shared_ptr<Polygon> CreateGrownPolygon(double padding) const;
    
    /**
       Determines the largest distance from the origin to any point of
       the polygon.
    */
    double CalculateRadius() const;
    
    size_t GetNPoints() const { return m_point.size(); }
    
    /**
       Convenience method for legacy code.
       
       \return A copy of the line between the corners (index) and
       (index + 1). The last point is connected with the first one.
    */
    boost::shared_ptr<Line> _GetLine(size_t index) const;
    
    /** \return Pointer to the point at given index, or 0 if invalid index. */
    const Point * GetPoint(size_t index) const;
    
    /**
       Writes the corners in human readable format on the provided ostream.
    */
    friend std::ostream & operator << (std::ostream & os, const Polygon & p);

    
  private:
    typedef std::vector<boost::shared_ptr<Point> > pointlist_t;
    
    pointlist_t m_point;
  };
  
}

#endif // SUNFLOWER_POLYGON_HPP
