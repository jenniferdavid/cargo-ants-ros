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


#ifndef BRAITENBERG_H
#define BRAITENBERG_H



#include <npm/common/Robot.hpp>



class Braitenberg:
  public Robot
{
public:
  Braitenberg(const std::string & name,
	      const World & world,
	      double timestep);
  ~Braitenberg();

  virtual void PrepareAction();
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual void SetGoal(const sfl::Goal & goal);
  virtual const sfl::Goal & GetGoal();
  virtual bool GoalReached();

private:
  class Sharp *left, *right;
  class DiffDrive *drive;
  class SharpDrawing *left_drawing, *right_drawing;
  class Goal &goal;
};

#endif // BRAITENBERG_H
