/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#include "asl_hal.hpp"
// this depends on --with-robox=/path/to/robox when configuring
#include "hal.h"
#include <sfl/util/numeric.hpp>


using namespace sfl;


namespace asl {
  
  
  int HAL::
  time_get(struct ::timespec * stamp)
  {
    return hal_time_get(stamp);
  }


  int HAL::
  odometry_set(double x, double y, double theta,
	       double sxx, double syy, double stt,
	       double sxy, double sxt, double syt)
  {
    struct hal_odometry_data_s data;
    if(0 != hal_time_get(&data.timestamp))
      return -42;
    data.x = x;
    data.y = y;
    data.theta = theta;
    data.cov_matrix[0][0] = sxx;
    data.cov_matrix[0][1] = sxy;
    data.cov_matrix[0][2] = sxt;
    data.cov_matrix[1][0] = sxy;
    data.cov_matrix[1][1] = syy;
    data.cov_matrix[1][2] = syt;
    data.cov_matrix[2][0] = sxt;
    data.cov_matrix[2][1] = syt;
    data.cov_matrix[2][2] = stt;
    return hal_odometry_set(data);
  }


  int HAL::
  odometry_get(struct ::timespec * stamp,
	       double * x, double * y, double * theta,
	       double * sxx, double * syy, double * stt,
	       double * sxy, double * sxt, double * syt)
  {
    struct hal_odometry_data_s data;
    const int status(hal_odometry_get(&data));
    if(0 != status)
      return status;
    *stamp = data.timestamp;
    *x = data.x;
    *y = data.y;
    *theta = data.theta;
    *sxx = data.cov_matrix[0][0];
    *syy = data.cov_matrix[1][1];
    *stt = data.cov_matrix[2][2];    
    *sxy = data.cov_matrix[0][1];
    *sxt = data.cov_matrix[0][2];
    *syt = data.cov_matrix[1][2];
    return 0;
  }


  int HAL::
  speed_set(double qdl, double qdr)
  {
    struct hal_speed_data_s data;
    if(0 != hal_time_get(&data.timestamp))
      return -42;
    data.speed_l = qdl;
    data.speed_r = qdr;
    return hal_speed_set(data);
  }


  int HAL::
  speed_get(double * qdl, double * qdr)
  {
    struct hal_speed_data_s data;
    const int status(hal_speed_get(&data));
    if(0 != status)
      return status;
    *qdl = data.speed_l;
    *qdr = data.speed_r;
    return 0;
  }


  int HAL::
  scan_get(int channel, double * rho, size_t * rho_len,
	   struct ::timespec * t0, struct ::timespec * t1)
  {
    const uint8_t hal_channel(static_cast<uint8_t>(channel & 0xFF));
    uint16_t nranges;
    if(0 != hal_laser_get_nranges(hal_channel, &nranges))
      return -42;
    hal_laser_range_t ranges[nranges];
    struct hal_laser_data_s data;
    data.nranges = nranges;
    data.ranges = ranges;
    const int status(hal_laser_get_channel(hal_channel, &data));
    if(0 != status)
      return status;
    *rho_len = minval(*rho_len, static_cast<size_t>(nranges));
    for(size_t ii(0); ii < *rho_len; ++ii)
      rho[ii] = ranges[ii] * 0.001; // robox HAL hardcodes mm resolution
    *t0 = data.timestamp;
    *t1 = data.timestamp;
    return 0;
  }
  
}
