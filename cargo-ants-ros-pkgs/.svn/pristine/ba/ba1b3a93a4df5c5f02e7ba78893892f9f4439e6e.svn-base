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

#ifndef CARGO_ANTS_TRACTOR_TRAILER_KINEMATICS_HPP
#define CARGO_ANTS_TRACTOR_TRAILER_KINEMATICS_HPP


namespace cargo_ants {
  
  
  /**
     See [lamiraux:1999] (Lamiraux, Sekhavat, and Laumond. Motion
     Planning and Control for Hilare Pulling a Trailer. IEEE
     Transaction on Robotics and Automation 15(4), August 1999.)
  */
  class TractorTrailerKinematics
  {
  public:
    TractorTrailerKinematics (/** l_r in [lamiraux:1999] */
			      double hitch_offset,
			      /** l_t in [lamiraux:1999] */
			      double trailer_arm);

    double const hitch_offset_;
    double const trailer_arm_;
    
    double pose_x_;
    double pose_y_;
    double pose_heading_;
    
    /** called phi in [lamiraux:1999] */
    double trailer_angle_;
    
    void computeUpdate (double v_trans, double v_rot, double dt);
  };
  
}

#endif // CARGO_ANTS_TRACTOR_TRAILER_KINEMATICS_HPP
