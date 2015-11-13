/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "PNFCamera.hpp"
#include "Esbot.hpp"
#include "PNF.hpp"
#include <npm/gfx/View.hpp>
#include <pnf/Flow.hpp>


using namespace npm;


PNFCamera::
PNFCamera(const std::string & name,
	  Esbot * bot)
  : Camera(name,
	   "fixed on grid size of a PNF instance"),
    m_bot(bot)
{
}


void PNFCamera::
ConfigureView(View & view)
{
  boost::shared_ptr<PNF> pnf(m_bot->GetPNF());
  if( ! pnf){
    view.SetRange(0, 1, 0, 1);
    return;
  }
  boost::shared_ptr<pnf::Flow> flow(pnf->GetFlow());
  view.SetRange(0, flow->xsize, 0, flow->ysize); // bizarre param order...
}
