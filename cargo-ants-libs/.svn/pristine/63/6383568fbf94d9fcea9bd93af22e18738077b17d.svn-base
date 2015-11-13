/* 
 * Copyright (C) 2005
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland.philippsen@gmx.net>
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


#ifndef DUMMY_HAL_HPP
#define DUMMY_HAL_HPP


#include <sfl/api/HAL.hpp>


class DummyHAL
  : public sfl::HAL
{
public:
  DummyHAL();

  virtual int time_get(struct ::timespec * stamp);
  virtual int odometry_set(double x, double y, double theta,
			   double sxx, double syy, double stt,
			   double sxy, double sxt, double syt);
  virtual int odometry_get(struct ::timespec * stamp,
			   double * x, double * y, double * theta,
			   double * sxx, double * syy, double * stt,
			   double * sxy, double * sxt, double * syt);
  virtual int speed_set(double qdl, double qdr);
  virtual int scan_get(int channel, double * rho, int rho_len,
		       struct ::timespec * t0, struct ::timespec * t1);
  
  double x, y, theta, sxx, syy, stt, sxy, sxt, syt;
  double qdl, qdr;
  double rho;
};

#endif // DUMMY_HAL_HPP
