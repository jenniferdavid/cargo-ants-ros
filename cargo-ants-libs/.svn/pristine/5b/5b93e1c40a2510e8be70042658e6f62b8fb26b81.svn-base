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


#include "DummyHAL.hpp"
#include <sys/time.h>


DummyHAL::
DummyHAL()
  : sxx(1), syy(1), stt(1), rho(1.5)
{
}


int DummyHAL::
time_get(struct timespec * stamp)
{
  struct timeval tv;
  int res(gettimeofday(&tv, 0));
  if(res != 0)
    return -1;
  TIMEVAL_TO_TIMESPEC(&tv, stamp);
  return 0;
}


int DummyHAL::
odometry_set(double _x, double _y, double _theta,
	     double _sxx, double _syy, double _stt,
	     double _sxy, double _sxt, double _syt)
{
  x = _x;
  y = _y;
  theta = _theta;
  sxx = _sxx;
  syy = _syy;
  stt = _stt;
  sxy = _sxy;
  sxt = _sxt;
  syt = _syt;
  return 0;
}


int DummyHAL::
odometry_get(struct timespec * stamp,
	     double * _x, double * _y, double * _theta,
	     double * _sxx, double * _syy, double * _stt,
	     double * _sxy, double * _sxt, double * _syt)
{
  int res(time_get(stamp));
  if(res != 0)
    return res;
  *_x = x;
  *_y = y;
  *_theta = theta;
  *_sxx = sxx;
  *_syy = syy;
  *_stt = stt;
  *_sxy = sxy;
  *_sxt = sxt;
  *_syt = syt;
  return 0;
}


int DummyHAL::
speed_set(double _qdl, double _qdr)
{
  qdl = _qdl;
  qdr = _qdr;
  return 0;
}


int DummyHAL::
scan_get(int channel, double * _rho, int rho_len,
	 struct timespec * t0, struct timespec * t1)
{
  int res(time_get(t0));
  if(res != 0)
    return res;
  for(int is(0); is < rho_len; ++is){
    _rho[is] = rho;
  }
  return time_get(t1);
}
