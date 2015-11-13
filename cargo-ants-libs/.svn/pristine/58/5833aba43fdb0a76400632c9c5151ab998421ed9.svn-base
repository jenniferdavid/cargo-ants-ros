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


#include "Braitenberg.hpp"
#include <util/Line.hpp>
#include <util/Goal.hpp>
#include "../common/DiffDrive.hpp"
#include "../common/Sharp.hpp"
#include "../common/SharpDrawing.hpp"
#include <cmath>



Braitenberg::
Braitenberg(const string &name,
	    double timestep):
  Robot(name, timestep, false),
  goal(* new Goal(Frame(0, 0, 0), 0, 0))
{
  left  = DefineSharp(Frame(0, 0,   M_PI / 4));
  right = DefineSharp(Frame(0, 0, - M_PI / 4));
  drive = DefineDiffDrive(0.15, 0.04);
  left_drawing  = new SharpDrawing("left_sharp", *left);
  right_drawing = new SharpDrawing("right_sharp", *right);

  AddLine(Line(-0.1, -0.1, -0.1,  0.1));
  AddLine(Line(-0.1,  0.1,  0.1,  0.0));
  AddLine(Line( 0.1,  0.0, -0.1, -0.1));
}



Braitenberg::
~Braitenberg()
{
  delete left_drawing;
  delete right_drawing;
  delete &goal;
}



void Braitenberg::
PrepareAction()
{
  drive->SetSpeed(2 * M_PI * left->Get()  / left->Rmax(),
		  2 * M_PI * right->Get() / right->Rmax());
}



void Braitenberg::
SetPose(double x, double y, double theta)
{
}



void Braitenberg::
GetPose(double &x, double &y, double &theta)
{
  x = 0;
  y = 0;
  theta = 0;
}



void Braitenberg::
SetGoal(const class Goal &goal)
{
}



Goal Braitenberg::
GetGoal()
{
  return goal;
}



bool Braitenberg::
GoalReached()
{
  return false;
}
