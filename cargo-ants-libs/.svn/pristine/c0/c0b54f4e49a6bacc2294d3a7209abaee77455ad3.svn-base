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


#include "TheaterRobotProxy.hpp"
#include "TheaterRobot.hpp"
#include "TheaterMotionManager.hpp"
#include <sfl/expo/MotionController.hpp>


TheaterRobotProxy::
TheaterRobotProxy(TheaterRobot & owner):
  _owner(owner),
  _clock(0),
  _mm(0),
  _monitor(0),
  _odometry(0),
  _motion_controller(0),
  _robot_model(0),
  _outstream(0),
  _sequencer(0)
{
}


const SynchronizedClock & TheaterRobotProxy::
GetClock()
  const
{
  if(_clock == 0){
    _clock = _owner.GetMutableClock();
  }
  return * _clock;
}


MotionManager & TheaterRobotProxy::
GetMotionManager()
  const
{
  if(_mm == 0){
    _mm = _owner.GetMotionManager();
  }
  return * _mm;
}


ProcessMonitor & TheaterRobotProxy::
GetMonitor()
  const
{
  if(_monitor == 0){
    _monitor = _owner.GetMonitor();
  }
  return * _monitor;
}


sfl::Odometry & TheaterRobotProxy::
GetOdometry()
  const
{
  if(_odometry == 0){
    _odometry = _owner.GetOdometry();
  }
  return * _odometry;
}


sfl::MotionController & TheaterRobotProxy::
GetMotionController()
  const
{
  if(_motion_controller == 0){
    _motion_controller = _owner.GetMotionController();
  }
  return * _motion_controller;
}


const sfl::RobotModel & TheaterRobotProxy::
GetRobotModel()
  const
{
  if(_robot_model == 0){
    _robot_model = _owner.GetMutableRobotModel();
  }
  return * _robot_model;
}


std::ostream & TheaterRobotProxy::
GetOutstream()
  const
{
  if(_outstream == 0){
    _outstream = _owner.GetOutstream();
  }
  return * _outstream;
}


Sequencer & TheaterRobotProxy::
GetSequencer()
  const
{
  if(_sequencer == 0){
    _sequencer = _owner.GetSequencer();
  }
  return * _sequencer;
}
