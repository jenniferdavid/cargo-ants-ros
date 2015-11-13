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


#ifndef THEATER_ROBOT_PROXY_HPP
#define THEATER_ROBOT_PROXY_HPP


#include <theater/RobotProxy.hpp>


class TheaterRobot;


namespace sfl {
  class Odometry;
  class MotionController;
  class RobotModel;
}


class TheaterRobotProxy:
  public RobotProxy
{
public:
  TheaterRobotProxy(TheaterRobot & owner);

  virtual const SynchronizedClock & GetClock() const;
  virtual MotionManager & GetMotionManager() const;
  virtual ProcessMonitor & GetMonitor() const;
  virtual std::ostream & GetOutstream() const;
  virtual Sequencer & GetSequencer() const;

  sfl::Odometry & GetOdometry() const;
  sfl::MotionController & GetMotionController() const;
  const sfl::RobotModel & GetRobotModel() const;


private:
  mutable TheaterRobot & _owner;
  
  mutable SynchronizedClock * _clock;
  mutable MotionManager * _mm;
  mutable ProcessMonitor * _monitor;
  mutable sfl::Odometry * _odometry;
  mutable sfl::MotionController * _motion_controller;
  mutable sfl::RobotModel * _robot_model;
  mutable std::ostream * _outstream;
  mutable Sequencer * _sequencer;
};

#endif // THEATER_ROBOT_PROXY_HPP
