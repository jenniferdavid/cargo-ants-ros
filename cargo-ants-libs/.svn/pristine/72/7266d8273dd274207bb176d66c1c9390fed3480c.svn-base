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


#ifndef SUNFLOWER_RAY_HPP
#define SUNFLOWER_RAY_HPP


#include <sfl/util/Frame.hpp>
#include <sfl/util/Line.hpp>


namespace sfl {


  /**
     Semi-infinite line.
  */
  class Ray
  {
  public:
    /**
       Construct a ray from point (x, y) into direction theta.
    */
    Ray(double x, double y, double theta);

    /**
       Construct a ray that corresponds to the given frame's x-axis.
    */
    Ray(const Frame & frame);

    /**
       Set the direction of the ray.
    */
    void SetAngle(double angle);

    /**
       Calculate the intersection between a Line and a Ray.
       
       Simply calls LineRayIntersect(), see util/numeric.hpp for more
       information.
    */
    double Intersect(const Line & line) const;

    /**
       \return X-coordinate of the ray's start.
    */
    inline double X() const;

    /**
       \return Y-coordinate of the ray's start.
    */
    inline double Y() const;

    /**
       \return X-component of the unit vector along the ray.
    */
    inline double Dx() const;

    /**
       \return Y-component of the unit vector along the ray.
    */
    inline double Dy() const;  


  protected:
    Frame _frame;
  };



  double Ray::
  X()
    const
  {
    return _frame.X();
  }



  double Ray::
  Y()
    const
  {
    return _frame.Y();
  }



  double Ray::
  Dx()
    const
  {
    return _frame.Costheta();
  }



  double Ray::
  Dy()
    const
  {
    return _frame.Sintheta();
  }



}

#endif // SUNFLOWER_RAY_HPP
