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

#ifndef NPM_EXT_ZOMBIE_HPP
#define NPM_EXT_ZOMBIE_HPP

#include <npm/RobotClient.hpp>

namespace npm {
  
  
  class Zombie:
    public RobotClient
  {
  public:
    Zombie(std::string const &name);
    
    virtual bool Initialize(RobotServer &server);
    virtual bool PrepareAction(double timestep);
    virtual void InitPose(sfl::Frame const &pose) {}
    virtual void SetPose(sfl::Frame const &pose) {}
    virtual bool GetPose(sfl::Frame &pose) const { return false; }
    virtual void SetGoal(double timestep, const sfl::Goal & goal);
    virtual bool GetGoal(sfl::Goal &goal) const;
    virtual bool GoalReached() const;
    
  protected:
    double m_width;
    double m_length;
    double m_thetad_max;
    double m_sd_max;
    
  private:
    RobotServer *m_server;
    boost::shared_ptr<HoloDrive> m_drive;
    sfl::Goal m_goal;
  };
  
  
  class LidarZombie:
    public Zombie
  {
  public:
    LidarZombie(std::string const &name);
    
    virtual bool Initialize(RobotServer &server);
    virtual bool PrepareAction(double timestep);
    
  protected:
    boost::shared_ptr<Lidar> m_lidar;
  };

}

#endif // NPM_EXT_ZOMBIE_HPP
