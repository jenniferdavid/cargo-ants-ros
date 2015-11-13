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


#ifndef SUNFLOWER_FUNCTORS_HPP
#define SUNFLOWER_FUNCTORS_HPP


namespace sfl {


  /**
     Convenient deletion of collections using std::for_each(). For
     example:

     \code
     typedef std::vector<Point *> point_t;
     point_t _point;
     double x, y;
     while(CollectPoint(x, y))
       _point.push_back(new Point(x, y));
     DoSomethingWithTemporaryPoints(_point);
     // delete all Point instances in _point:
     for_each(_point.begin(), _point.end(), Deleter());
     \endcode
  */
  class Deleter
  {
  public:
    /**
       Function call semantics.
    */
    template <typename T>
    void operator()(T * pointer)
    {
      delete pointer;
    }
  };
  
}

#endif // SUNFLOWER_FUNCTORS_HPP
