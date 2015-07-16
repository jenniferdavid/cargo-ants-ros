/* Cargo-ANTs software prototype.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cargo_ants_util/util.h"
#include <Eigen/Geometry>
#include <stdio.h>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <vector>

void foo ()
{
  if (std::isinf(42.3)) {
    double x = 17;
  }
}


namespace cargo_ants_util {
  
  
  Eigen::Vector3d pose3Dto2D (geometry_msgs::Point const & translation,
			      geometry_msgs::Quaternion const & orientation)
  {
    // Can somebody please explain to me why Eigen has chosen to store
    // the parameters in (x,y,z,w) order, but provide a ctor in
    // (w,x,y,z) order? What have they been smoking?
    //
    Eigen::Quaternion<double> const rot (orientation.w,
					 orientation.x,
					 orientation.y,
					 orientation.z);
    Eigen::Vector3d ux;
    ux << 1.0, 0.0, 0.0;
    ux = rot._transformVector (ux);
    Eigen::Vector3d pose (translation.x, translation.y, atan2 (ux.y(), ux.x()));
    return pose;
  }
  
  
  double mod2pi (double aa)
  {
    aa = fmod(aa, 2.0 * M_PI);
    if (aa > M_PI) {
      return aa - 2.0 * M_PI;
    }
    if (aa <= - M_PI) {
      return aa + 2.0 * M_PI;
    }
    return aa;
  }
  
  
  bool goalReached (cargo_ants_msgs::Goal const & goal,
		    Eigen::Vector3d const & pose,
		    bool go_forward)
  {
    if (goal.dr > 0) {
      double const dx (goal.gx - pose[0]);
      double const dy (goal.gy - pose[1]);
      if (sqrt (dx * dx + dy * dy) > goal.dr) {
	return false;
      }
    }
    
    if ((goal.dth > 0.0) && (goal.dth < M_PI)) {
      double dth;
      if (go_forward) {
	dth = mod2pi (goal.gth - pose[2]);
      }
      else {
	dth = mod2pi (M_PI + pose[2] - goal.dth);
      }
      if (fabs (dth) > goal.dth) {
	return false;
      }
    }
    
    return true;
  }
  
  
  void prettyPrint (double vv, std::ostream & os)
  {
    static int const buflen (32);
    char buf [buflen];
    memset (buf, 0, sizeof(buf));

#ifndef WIN32
    if (std::isinf (vv)) {
      snprintf (buf, buflen-1, " inf    ");
    }
    else if (std::isnan (vv)) {
      snprintf (buf, buflen-1, " nan    ");
    }
    else if (fabs (fmod (vv, 1)) < 1e-6) {
      snprintf (buf, buflen-1, "%- 7d  ", static_cast <int> (rint (vv)));
    }
    else {
      snprintf (buf, buflen-1, "% 6.4f  ", vv);
    }
#else // WIN32
    sprintf_s (buf, buflen-1, "% 6.4f  ", vv);
#endif // WIN32
    
    os << buf;
  }
  
  
  void prettyPrint (Eigen::MatrixXd const & mm, std::ostream & os,
		    std::string const & title, std::string const & prefix,
		    bool vecmode, bool nonl)
  {
    char const * nlornot ("\n");
    if (nonl) {
      nlornot = "";
    }
    if ( ! title.empty()) {
      os << title << nlornot;
    }
    if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
      os << prefix << " (empty)" << nlornot;
    }
    else {
      
      if (vecmode) {
	if ( ! prefix.empty()) {
	  os << prefix;
	}
	for (int ir (0); ir < mm.rows(); ++ir) {
	  prettyPrint (mm.coeff (ir, 0), os);
	}
	os << nlornot;
	
      }
      else {

	for (int ir (0); ir < mm.rows(); ++ir) {
	  if ( ! prefix.empty()) {
	    os << prefix;
	  }
	  for (int ic (0); ic < mm.cols(); ++ic) {
	    prettyPrint (mm.coeff (ir, ic), os);
	  }
	  os << nlornot;
	}
	
      }
    }
  }
  
  
  void prettyPrint (Eigen::VectorXd const & vv, std::ostream & os,
		    std::string const & title, std::string const & prefix,
		    bool nonl)
  {
    prettyPrint ((Eigen::MatrixXd const &) vv, os, title, prefix, true, nonl);
  }
  
  
  void prettyPrint (Eigen::Vector3d const & vv, std::ostream & os,
		    std::string const & title, std::string const & prefix,
		    bool nonl)
  {
    prettyPrint ((Eigen::MatrixXd const &) vv, os, title, prefix, true, nonl);
  }
  
}
