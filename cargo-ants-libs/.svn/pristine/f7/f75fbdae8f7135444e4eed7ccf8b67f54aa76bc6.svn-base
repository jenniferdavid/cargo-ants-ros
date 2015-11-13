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


#include "GridLayerCamera.hpp"
#include <npm/gfx/View.hpp>
#include <sfl/gplan/NF1.hpp>


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


GridLayerCamera::
GridLayerCamera(const string & name, const NF1 & _nf1)
  : Camera(name,
	   "grid range for locally plotting NF1 and such"),
    nf1(_nf1)
{
}


void GridLayerCamera::
ConfigureView(View & view)
{
  shared_ptr<const NF1::grid_t> grid(nf1.GetGridLayer());
  view.SetBounds (0, 0, grid->xsize, grid->ysize);
}
