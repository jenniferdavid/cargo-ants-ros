/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#ifndef ASL_ARC_DRAWING_HPP
#define ASL_ARC_DRAWING_HPP


#include <npm/common/Drawing.hpp>
#include <npm/common/World.hpp>


class Smart;


class ArcDrawing
  : public npm::Drawing,
    public npm::KeyListener
{
public:
  ArcDrawing(const std::string & name,
	     /** \note could be const, if only Smart::GetPose() was const */
	     Smart * smart,
	     bool enabled,
	     bool recomp_status);
  
  virtual void Draw();
  virtual void KeyPressed(unsigned char key);
  
private:
  Smart * m_smart;
  bool m_enabled;
  bool m_recomp_status;
};

#endif // ASL_ARC_DRAWING_HPP
