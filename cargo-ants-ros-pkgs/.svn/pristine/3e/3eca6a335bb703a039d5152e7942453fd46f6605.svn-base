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

#include "TractorTrailerKinematics.hpp"

namespace cargo_ants {
  
  
  TractorTrailerKinematics::
  TractorTrailerKinematics (double hitch_offset,
			    double trailer_arm)
    : hitch_offset_ (hitch_offset),
      trailer_arm_ (trailer_arm),
      pose_x_ (0.0),
      pose_y_ (0.0),
      pose_heading_ (0.0),
      trailer_angle_ (0.0)
  {
  }

  
  void TractorTrailerKinematics::
  computeUpdate (double v_trans, double v_rot, double dt)
  {
    double const cth (cos (pose_heading_));
    double const sth (sin (pose_heading_));
    
    double const dx (dt * vtrans * cth);
    double const dy (dt * vtrans * sth);

    pose_x_ += dx * cth - dy * sth;
    pose_y_ += dy * cth + dx * sth;
    pose_heading_ += dt * vrot;
    
    trailer_angle_ -=
      dt * (vtrans * sin (trailer_angle_)
	    + vrot * (hitch_offset_ * cos (trailer_angle_) + 1.0))
      / trailer_arm_;
  }
  
}
