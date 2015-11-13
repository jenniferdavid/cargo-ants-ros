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


#include "Frame.hpp"
#include "numeric.hpp"
#include <sfl/api/Pose.hpp>
#include <sfl/api/Goal.hpp>
#include <cmath>
#include <iostream>


namespace sfl {


  Frame::
  Frame()
    : m_x(0), m_y(0), m_theta(0), m_sintheta(0), m_costheta(1)
  {
  }


  Frame::
  Frame(double x, double y, double theta)
    : m_x(x), m_y(y), m_theta(mod2pi(theta)),
      m_sintheta(sin(theta)), m_costheta(cos(theta))
  {
  }


  void Frame::
  Set(const Frame & f)
  {
    m_x = f.m_x;
    m_y = f.m_y;
    m_theta = f.m_theta;
    m_sintheta = f.m_sintheta;
    m_costheta = f.m_costheta;
  }


  void Frame::
  Set(double x, double y, double theta)
  {
    m_x = x;
    m_y = y;
    m_theta = mod2pi(theta);
    m_sintheta = sin(theta);
    m_costheta = cos(theta);
  }


  void Frame::
  Get(double & x, double & y, double & theta) const
  {
    x = m_x;
    y = m_y;
    theta = m_theta;
  }


  double Frame::
  X() const
  {
    return m_x;
  }


  double Frame::
  Y() const
  {
    return m_y;
  }


  double Frame::
  Theta() const
  {
    return m_theta;
  }


  double Frame::
  Costheta() const
  {
    return m_costheta;
  }


  double Frame::
  Sintheta() const
  {
    return m_sintheta;
  }


  void Frame::
  To(double & x, double & y) const
  {
    double tmpx(x * m_costheta - y * m_sintheta);
    double tmpy(x * m_sintheta + y * m_costheta);
    x = tmpx + m_x;
    y = tmpy + m_y;
  }


  void Frame::
  To(Frame & frame) const
  {
    To(frame.m_x, frame.m_y);
    RotateTo(frame.m_costheta, frame.m_sintheta);
    frame.m_theta = mod2pi(frame.m_theta + m_theta);
  }


  void Frame::
  To(Goal & goal) const
  {
    To(goal._x, goal._y, goal._theta);
  }


  void Frame::
  To(double & x, double & y, double & theta) const
  {
    To(x, y);
    theta = mod2pi(theta + m_theta);
  }


  void Frame::
  RotateTo(double & x, double & y) const
  {
    double tmpx(x * m_costheta - y * m_sintheta);
    double tmpy(x * m_sintheta + y * m_costheta);
    x = tmpx;
    y = tmpy;
  }


  void Frame::
  RotateTo(Frame & frame) const
  {
    RotateTo(frame.m_x, frame.m_y);
    RotateTo(frame.m_costheta, frame.m_sintheta);
    frame.m_theta = mod2pi(frame.m_theta + m_theta);
  }


  void Frame::
  From(double & x, double & y) const
  {
    x -= m_x;
    y -= m_y;
    double tmpx( x * m_costheta + y * m_sintheta);
    double tmpy(-x * m_sintheta + y * m_costheta);
    x = tmpx;
    y = tmpy;
  }


  void Frame::
  From(Frame & frame) const
  {
    From(frame.m_x, frame.m_y);
    RotateFrom(frame.m_costheta, frame.m_sintheta);
    frame.m_theta = mod2pi(frame.m_theta - m_theta);
  }


  void Frame::
  From(Goal & goal) const
  {
    From(goal._x, goal._y, goal._theta);
  }


  void Frame::
  From(double & x, double & y, double & theta) const
  {
    From(x, y);
    theta = mod2pi(theta - m_theta);
  }


  void Frame::
  RotateFrom(double & x, double & y) const
  {
    double tmpx(   x * m_costheta + y * m_sintheta);
    double tmpy( - x * m_sintheta + y * m_costheta);
    x = tmpx;
    y = tmpy;
  }


  void Frame::
  RotateFrom(Frame & frame) const
  {
    RotateFrom(frame.m_x, frame.m_y);
    RotateFrom(frame.m_costheta, frame.m_sintheta);
    frame.m_theta = mod2pi(frame.m_theta - m_theta);
  }


  void Frame::
  Add(double dx, double dy, double dtheta)
  {
    m_x += dx;
    m_y += dy;
    m_theta = mod2pi(m_theta + dtheta);
    m_costheta = cos(m_theta);
    m_sintheta = sin(m_theta);
  }


  std::ostream & operator << (std::ostream & os, const Frame & f)
  {
    os << "(" << f.m_x <<  ", " <<  f.m_y <<  ", " <<  f.m_theta <<  ")";
    return os;
  }


  Frame & Frame::
  operator = (const Frame & orig)
  {
    Set(orig);
    return * this;
  }


  Frame & Frame::
  operator = (const Pose & orig)
  {
    Set(static_cast<const Frame &>(orig));
    return * this;
  }

  double Frame::
  DistanceDiff(const Frame& compareFrame) const
  {
	return sqrt((X() - compareFrame.X()) * (X() - compareFrame.X())
			+ (Y() - compareFrame.Y()) * (Y() - compareFrame.Y()));
  }

  double Frame::
  AngleDiff(const Frame& compareFrame) const
  {
	return sfl::absval(sfl::mod2pi(Theta()
			- compareFrame.Theta()));;
  }

}
