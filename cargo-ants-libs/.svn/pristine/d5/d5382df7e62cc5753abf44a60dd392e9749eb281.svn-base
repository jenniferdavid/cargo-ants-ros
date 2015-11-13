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


#ifndef ROBOX_HPP
#define ROBOX_HPP

#include <npm/RobotClient.hpp>
#include <npm/ext/expo02/robox_parameters.hpp>
#include <map>

namespace sfl {
  class Hull;
}

namespace npm {
  class VisualRobox;
}

namespace local {
  class NGKeyListener;
}

struct scanner_desc_s {
  /** Defaults like the front scanner of Robox. */
  scanner_desc_s();
  int nscans;
  double mount_x, mount_y, mount_theta;
  double rhomax, phi0, phirange;
};


class Robox
  : public npm::RobotClient
{
public:
  /** \todo XXXX to-do: resurrect customizability (the old
      Robox::CreateCustom factory method) using
      fpplib::Configurable. See commented-out sections of
      Robox.cpp. */
  Robox(std::string const &name);
  
  virtual bool Initialize(npm::RobotServer &server);
  virtual bool PrepareAction(double timestep);
  virtual void InitPose(sfl::Frame const &pose);
  virtual void SetPose(sfl::Frame const &pose);
  virtual bool GetPose(sfl::Frame &pose) const;
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual bool GetGoal(sfl::Goal &goal) const;
  virtual bool GoalReached() const;
  
protected:
  robox_parameters m_params;
  
  boost::shared_ptr<npm::VisualRobox> m_imp;
  boost::shared_ptr<npm::DiffDrive> m_drive;
  boost::shared_ptr<local::NGKeyListener> m_ngkl;
};

#endif // ROBOX_HPP
