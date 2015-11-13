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


#include "StillCamera.hpp"
#include "View.hpp"
#include <sfl/util/strutil.hpp>


using namespace sfl;
using namespace std;


namespace npm {
  
  
  StillCamera::
  StillCamera(const string & name,
	      double _x0, double _y0,
	      double _x1, double _y1)
    : Camera(name,
	     "fixed camera (" + to_string(_x0) + " " + to_string(_y0)
	     + " " + to_string(_x1) + " " + to_string(_y1) + " " ")"),
      x0(_x0),
      y0(_y0),
      x1(_x1),
      y1(_y1)
  {
  }
  
  
  void StillCamera::
  ConfigureView(View & view)
  {
    view.SetBounds (x0, y0, x1, y1);
  }
  
}
