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


#ifndef SUNFLOWER_ASL_HAL_HPP
#define SUNFLOWER_ASL_HAL_HPP

#include <sfl/api/HAL.hpp>

namespace asl {
  
  class HAL
    : public sfl::HAL
  {
  public:
    //    virtual ~HAL() { }
    
    /** \return 0 on success. */
    virtual int time_get(struct ::timespec * stamp);
    
    /** \return 0 on success. */
    virtual int odometry_set(double x, double y, double theta,
			     double sxx, double syy, double stt,
			     double sxy, double sxt, double syt);
    
    /** \return 0 on success. */
    virtual int odometry_get(struct ::timespec * stamp,
			     double * x, double * y, double * theta,
			     double * sxx, double * syy, double * stt,
			     double * sxy, double * sxt, double * syt);
    
    /** \return 0 on success. */
    virtual int speed_set(double qdl, double qdr);
    
    /** \return 0 on success. */
    virtual int speed_get(double * qdl, double * qdr);
    
    /** \return 0 on success. */
    virtual int scan_get(int channel, double * rho, size_t * rho_len,
			 struct ::timespec * t0, struct ::timespec * t1);
  };
  
}

#endif // SUNFLOWER_ASL_HAL_HPP
