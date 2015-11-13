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


#include "OCamera.hpp"
#include <npm/gfx/View.hpp>
#include <sfl/dwa/DynamicWindow.hpp>


using namespace npm;


OCamera::
OCamera(const std::string & name,
	const sfl::DynamicWindow & dwa)
  : Camera(name,
	   "fixed on the range of a DWA objective grid"),
    m_dwa(dwa)
{
}


void OCamera::
ConfigureView(View & view)
{
  view.SetBounds (m_dwa.QdlMinIndex(),
		  m_dwa.QdrMinIndex(),
		  m_dwa.QdlMaxIndex() + 1,
		  m_dwa.QdrMaxIndex() + 1);
}
