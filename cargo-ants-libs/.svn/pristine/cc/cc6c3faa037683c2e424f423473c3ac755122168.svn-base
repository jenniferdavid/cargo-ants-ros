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


#include "TheaterRobotParameters.hpp"


//using namespace sunflower;


TheaterRobotParameters::
TheaterRobotParameters():
  _front_mount(- 0.035, 0, 0),
  _rear_mount( - 0.174, 0, M_PI)
{
}


TheaterRobotParameters * TheaterRobotParameters::
CreateButler()
{
  TheaterRobotParameters * trp(new TheaterRobotParameters());
  
  trp->_outline.AddPoint(-0.160,  0.364);
  trp->_outline.AddPoint(-0.064,  0.383);
  trp->_outline.AddPoint( 0.064,  0.383);
  trp->_outline.AddPoint( 0.160,  0.364);
  trp->_outline.AddPoint( 0.160, -0.364);
  trp->_outline.AddPoint( 0.064, -0.383);
  trp->_outline.AddPoint(-0.064, -0.383);
  trp->_outline.AddPoint(-0.160, -0.364);
  trp->_outline.AddPoint(-0.160, -0.154);
  trp->_outline.AddPoint(-0.609, -0.154);
  trp->_outline.AddPoint(-0.650, -0.064);
  trp->_outline.AddPoint(-0.650,  0.064);
  trp->_outline.AddPoint(-0.609,  0.154);
  trp->_outline.AddPoint(-0.160,  0.154);

  Polygon front;
  front.AddPoint(-0.160,  0.364);
  front.AddPoint(-0.064,  0.383);
  front.AddPoint( 0.064,  0.383);
  front.AddPoint( 0.160,  0.364);
  front.AddPoint( 0.160, -0.364);
  front.AddPoint( 0.064, -0.383);
  front.AddPoint(-0.064, -0.383);
  front.AddPoint(-0.160, -0.364);
  trp->_hull.AddPolygon(front);
  
  Polygon rear;
  rear.AddPoint(-0.650,  0.064);
  rear.AddPoint(-0.609,  0.154);
  rear.AddPoint(-0.160,  0.154);
  rear.AddPoint(-0.160, -0.154);
  rear.AddPoint(-0.609, -0.154);
  rear.AddPoint(-0.650, -0.064);
  trp->_hull.AddPolygon(rear);

  return trp;
}


TheaterRobotParameters * TheaterRobotParameters::
CreateDancer()
{
  return 0;
}


TheaterRobotParameters * TheaterRobotParameters::
CreateAnimal()
{
  return 0;
}


double TheaterRobotParameters::
SecurityDistance()
  const
{
  return 0.02;
}

double TheaterRobotParameters::
WheelBase()
  const
{
  return 0.6983;
}


double TheaterRobotParameters::
WheelRadius()
  const
{
  return 0.155;
}


double TheaterRobotParameters::
QdMax()
  const
{
  return 1.1 * SdMax() / WheelRadius();
}


double TheaterRobotParameters::
QddMax()
  const
{
  return 3 * QdMax();
}


double TheaterRobotParameters::
SdMax()
  const
{
  return 2.0;
}


double TheaterRobotParameters::
ThetadMax()
  const
{
  return 8.0;
}


const Hull & TheaterRobotParameters::
GetHull()
  const
{
  return _hull;
}


const Polygon & TheaterRobotParameters::
GetOutline()
  const
{
  return _outline;
}


const Frame & TheaterRobotParameters::
FrontLidarMount()
  const
{
  return _front_mount;
}


const Frame & TheaterRobotParameters::
RearLidarMount()
  const
{
  return _rear_mount;
}
