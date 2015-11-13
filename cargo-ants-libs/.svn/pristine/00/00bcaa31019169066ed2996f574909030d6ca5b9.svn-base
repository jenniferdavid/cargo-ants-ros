/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "ScannerDrawing.hpp"
#include <npm/RobotServer.hpp>
#include <npm/Lidar.hpp>
#include "wrap_glu.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/api/Scan.hpp>


using namespace std;
using namespace sfl;


namespace npm {
  
  
  ScannerDrawing::
  ScannerDrawing(const Lidar * lidar)
    : Drawing(lidar->owner->GetName() + "_" + lidar->name,
	      "laser scanner data in global reference frame"),
      m_lidar(lidar)
  {
  }
  
  
  void ScannerDrawing::
  Draw()
  {
    vector<double> rho, goodx, goody;
    Frame const & sensor_pose (m_lidar->GetGlobalPose());
    double const dphi(m_lidar->phirange / (m_lidar->nscans - 1));
    
    double phi(sensor_pose.Theta() + m_lidar->phi0);
    Frame ray(sensor_pose.X(), sensor_pose.Y(), phi);

    glLineWidth(1);
    glColor3d(0.8, 0.4, 0);
    glBegin(GL_LINE_LOOP);
    glVertex2d(sensor_pose.X(), sensor_pose.Y());
    for(size_t ir(0); ir < m_lidar->nscans; ++ir){
      double xx(m_lidar->GetNoisyRho(ir));
      double yy(0.0);
      ray.To(xx, yy);
      glVertex2d(xx, yy);
      
      if (m_lidar->GetNoisyRho(ir) < m_lidar->rhomax) {
	goodx.push_back(xx);
	goody.push_back(yy);
      }
      
      phi += dphi;
      ray.Set(ray.X(), ray.Y(), phi);
    }
    glEnd();
    
    if( ! goodx.empty()){
      glColor3d(1, 0.5, 0);
      glPointSize(3);
      glBegin(GL_POINTS);
      for(size_t ii(0); ii < goodx.size(); ++ii)
	glVertex2d(goodx[ii], goody[ii]);
      glEnd();
    }
  }
  
}
