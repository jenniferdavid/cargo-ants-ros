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

#include "Zombie.hpp"
#include <npm/HoloDrive.hpp>
#include <npm/RobotServer.hpp>
#include <npm/Lidar.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Scanner.hpp>

using namespace sfl;
using namespace boost;

namespace npm {
  
  
  Zombie::
  Zombie(std::string const &name)
    : RobotClient(name),
      m_width(0.6),
      m_length(0.3),
      m_thetad_max(0.8 * M_PI),
      m_sd_max(0.4),
      m_server(0)
  {
    reflectParameter("width", &m_width);
    reflectParameter("length", &m_length);
    reflectParameter("thetad_max", &m_thetad_max);
    reflectParameter("sd_max", &m_sd_max);
  }
  
  
  bool Zombie::
  Initialize(RobotServer &server)
  {
    if ( !RobotClient::Initialize(server)) {
      return false;
    }
    
    m_server = &server;		// as a "cheat" so we don't need localization etc
    
    m_drive = server.DefineHoloDrive(0.6);
    
    server.AddLine(Line(-m_length/2.0, -m_width/2.0, -m_length/2.0,  m_width/2.0));
    server.AddLine(Line(-m_length/2.0,  m_width/2.0,  m_length/2.0,  m_width/2.0));
    server.AddLine(Line( m_length/2.0,  m_width/2.0,  m_length/2.0, -m_width/2.0));
    server.AddLine(Line( m_length/2.0, -m_width/2.0, -m_length/2.0, -m_width/2.0));
    
    return true;
  }
  
  
  bool Zombie::
  PrepareAction(double timestep)
  {
    const Frame & pose(m_server->GetPose());
    double dx(m_goal.X() - pose.X());
    double dy(m_goal.Y() - pose.Y());
    pose.RotateFrom(dx, dy);
    double dtheta(atan2(dy, dx));
    
    double thetad(dtheta / timestep);
    double xd(sqrt(dx * dx + dy * dy) / timestep);
    double yd(0);
    
    // if(absval(dtheta) > dthetathresh)
    //   xd = 0;
    
    m_drive->vx = boundval(-m_sd_max, xd, m_sd_max);
    m_drive->vy = boundval(-m_sd_max, yd, m_sd_max);
    m_drive->omega = boundval(-m_thetad_max, thetad, m_thetad_max);
    
    return true;
  }
  
  
  void Zombie::
  SetGoal(double timestep, const Goal & goal)
  {
    m_goal = goal;
  }
  
  
  bool Zombie::
  GetGoal(Goal &goal)
    const
  {
    goal = m_goal;
    return true;
  }
  
  
  bool Zombie::
  GoalReached()
    const
  {
    return m_goal.DistanceReached(m_server->GetPose());
  }
  
  
  LidarZombie::
  LidarZombie(std::string const &name)
    : Zombie(name)
  {
  }
  
  
  bool LidarZombie::
  Initialize(RobotServer &server)
  {
    if ( !Zombie::Initialize(server)) {
      return false;
    }
    m_lidar = server.DefineLidar(Frame(0.0, 0.0, 0.0),
				 "lidar",
				 361, 8.0, -M_PI/2, M_PI);
    return true;
  }
  
  
  /** \todo Can probably be removed... */
  bool LidarZombie::
  PrepareAction(double timestep)
  {
    return Zombie::PrepareAction(timestep);
  }
  
}
