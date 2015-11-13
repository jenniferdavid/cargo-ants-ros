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


#ifndef NPM_STILLCAMERA_HPP
#define NPM_STILLCAMERA_HPP


#include <npm/gfx/Camera.hpp>


namespace npm {
  
  /**
     A fixed-bounding-box Camera. In case no fancy bounding-box
     modifications need to be made each cycle, this is the camera to
     use. You can still redefine the range using
     StillCamera::SetRange(), but this has to be called explicitly.
  */
  class StillCamera
    : public Camera
  {
  public:
    StillCamera(const std::string & name,
		double x0, double y0,
		double x1, double y1);
    virtual void ConfigureView(View & view);
    double x0, y0, x1, y1;
  };

}

#endif // NPM_STILLCAMERA_HPP
