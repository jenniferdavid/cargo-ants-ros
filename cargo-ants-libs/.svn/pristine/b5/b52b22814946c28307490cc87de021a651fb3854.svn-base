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


#ifndef RHDRAWING_H
#define RHDRAWING_H


#include <npm/gfx/Drawing.hpp>
#include <sfl/bband/ReplanHandler.hpp>
#include <string>


class RHDrawing
  : public npm::Drawing
{
public:
  typedef enum { BAND, INITIALBAND, AUTODETECT } mode_t;

  RHDrawing(const std::string & name,
	    const sfl::ReplanHandler * replan_handler,
	    mode_t mode);

  void Draw();

private:
  const sfl::ReplanHandler * _replan_handler;
  mode_t _mode;

  void DrawBand();
};

#endif // RHDRAWING_H
