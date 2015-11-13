/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "RobotClient.hpp"
#include "RobotServer.hpp"
#include <sfl/util/Frame.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/Polygon.hpp>
#include <boost/bind.hpp>

using namespace sfl;
using namespace boost;


namespace npm {

  RobotClient::registry_t RobotClient::registry;
  
  
  RobotClient::
  RobotClient(std::string const &name)
    : fpplib::Configurable(name),
      m_enable_trajectory(true),
      m_noisy_scanners(false),
      m_scanner_noise_min_factor(-1), // factors <0 are ignored
      m_scanner_noise_max_factor(-1),
      m_scanner_noise_min_offset(-0.1), // if min>max then offsets are ignored
      m_scanner_noise_max_offset( 0.1),
      m_camera_zoom(2),
      m_color(1.0, 1.0, 1.0)
  {
    registry.add(name, this);
    reflectParameter("enable_trajectory", &m_enable_trajectory);
    reflectParameter("noisy_scanners", &m_noisy_scanners);
    reflectParameter("scanner_noise_min_factor", &m_scanner_noise_min_factor);
    reflectParameter("scanner_noise_max_factor", &m_scanner_noise_max_factor);
    reflectParameter("scanner_noise_min_offset", &m_scanner_noise_min_offset);
    reflectParameter("scanner_noise_max_offset", &m_scanner_noise_max_offset);
    reflectParameter("camera_zoom", &m_camera_zoom);
    reflectCallback<qhgoal_s>("goals", true, boost::bind(&RobotClient::AppendGoal, this, _1));
    reflectParameter("pose", &m_initial_pose);
    reflectParameter("color", &m_color);
  }
  
  
  bool RobotClient::
  Initialize(RobotServer &server)
  {
    return true;
  }
  
  
  // void RobotClient::
  // AddPolygon(const Polygon & polygon)
  // {
  //   size_t const npoints(polygon.GetNPoints());
  //   if (npoints < 2)
  //     return;
  //   Point const * pt0(polygon.GetPoint(npoints - 1));
  //   Point const * pt1(polygon.GetPoint(0));
  //   m_server->AddLine(Line(pt0->X(), pt0->Y(), pt1->X(), pt1->Y()));
  //   for (size_t ii(1); ii < npoints; ++ii) {
  //     pt0 = pt1;
  //     pt1 = polygon.GetPoint(ii);
  //     m_server->AddLine(Line(pt0->X(), pt0->Y(), pt1->X(), pt1->Y()));
  //   }
  // }
  
  
  bool RobotClient::
  AppendGoal(qhgoal_s const &goal)
  {
    m_goals.push_back(sfl::Goal(goal.x, goal.y, goal.theta, goal.dr, goal.dtheta));
    return true;
  }
  
}


namespace std {
  
  ostream & operator << (ostream &os, npm::qhgoal_s const &rhs)
  {
    return os << "(" << rhs.x << ", " << rhs.y << ", " << rhs.theta << ", " << rhs.dr << ", " << rhs.dtheta << ")";
  }
  
  ostream & operator << (ostream &os, npm::qhpose_s const &rhs)
  {
    return os << "(" << rhs.x << ", " << rhs.y << ", " << rhs.theta << ")";
  }
  
  ostream & operator << (ostream &os, npm::color_s const &rhs)
  {
    return os << "(" << rhs.red << ", " << rhs.green << ", " << rhs.blue << ")";
  }

}
