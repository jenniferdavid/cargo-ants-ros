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


#ifndef THEATER_ROBOT_PARAMETERS_HPP
#define THEATER_ROBOT_PARAMETERS_HPP


#include <sfl/api/RobotModel.hpp>


class TheaterRobotParameters:
  public sfl::RobotModel::Parameters
{
protected:
  TheaterRobotParameters();


public:
  static TheaterRobotParameters * CreateButler();
  static TheaterRobotParameters * CreateDancer();
  static TheaterRobotParameters * CreateAnimal();
  

  virtual double SecurityDistance() const;
  virtual double WheelBase() const;
  virtual double WheelRadius() const;
  virtual double QdMax() const;
  virtual double QddMax() const;
  virtual double SdMax() const;
  virtual double ThetadMax() const;
  virtual const sfl::Hull & GetHull() const;
  virtual const sfl::Polygon & GetOutline() const;
  
  const sfl::Frame & FrontLidarMount() const;
  const sfl::Frame & RearLidarMount() const;
  
  
protected:
  sfl::Hull _hull;
  sfl::Polygon _outline;
  sfl::Frame _front_mount;
  sfl::Frame _rear_mount;
};

#endif // THEATER_ROBOT_PARAMETERS_HPP
