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


#include "RobotZoomCamera.hpp"
#include <npm/RobotServer.hpp>
#include "View.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/util/Frame.hpp>


using namespace sfl;
using namespace std;


namespace npm {

  RobotZoomCamera::
  RobotZoomCamera(const RobotServer * robot, double radius)
    : Camera(robot->GetName() + "_true_zoom_camera",
	     "true robot pose +/- " + to_string(radius)),
      m_robot(robot),
      m_radius(radius)
  {
  }



  void RobotZoomCamera::
  ConfigureView(class View &view)
  {
    const Frame & pose(m_robot->GetPose());
    view.SetBounds (pose.X() - m_radius,
		    pose.Y() - m_radius,
		    pose.X() + m_radius,
		    pose.Y() + m_radius);
  }

}
