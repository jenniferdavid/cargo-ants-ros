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


#ifndef SUNFLOWER_POSE_HPP
#define SUNFLOWER_POSE_HPP


#include <sfl/util/Frame.hpp>
#include <sfl/api/Timestamp.hpp>


namespace sfl {


  /**
     A timestamped Frame with covariance information.
  */
  class Pose
    : public Frame
  {
  public:
    /** Default position = (0, 0, 0), timestamp = first, covariance = identity */
    Pose();
    
    /** Default timestamp = first, covariance = identity */
    Pose(double x, double y, double theta);

    void Set(double x, double y, double theta,
	     timespec_t tstamp,
	     double sxx, double syy, double stt,
	     double sxy, double sxt, double syt);
    
    Timestamp m_tstamp;
    double m_sxx, m_syy, m_stt, m_sxy, m_sxt, m_syt;
  };
  
}

#endif // SUNFLOWER_POSE_HPP
