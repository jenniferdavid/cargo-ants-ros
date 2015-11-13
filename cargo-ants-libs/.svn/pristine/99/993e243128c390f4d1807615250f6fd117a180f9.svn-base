/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006
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


#include "TraversabilityDrawing.hpp"
#include "wrap_gl.hpp"
#include "View.hpp"
#include <npm/BBox.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <math.h>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  TraversabilityDrawing::
  TraversabilityDrawing(const string & name,
												shared_ptr<TravProxyAPI> proxy,
												color_code_t _color_code)
    : Drawing(name,
							"traversability map as greyscale with special highlights"),
			color_code(_color_code),
			m_proxy(proxy)
  {
  }
  
  
  TraversabilityDrawing::
  TraversabilityDrawing(const string & name,
												TravProxyAPI * proxy,
												color_code_t _color_code)
    : Drawing(name,
							"traversability map as greyscale with special highlights"),
			color_code(_color_code),
			m_proxy(proxy)
  {
  }
  
  
  void TraversabilityDrawing::
  Draw()
  {
		if( ! m_proxy->Enabled())
			return;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(m_proxy->GetX(), m_proxy->GetY(), 0);
    glRotated(180 * m_proxy->GetTheta() / M_PI, 0, 0, 1);
    glScaled(m_proxy->GetDelta(), m_proxy->GetDelta(), 1);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		
		double const obstacle(m_proxy->GetObstacle());
		double const freespace(m_proxy->GetFreespace());
    double const cscale(1.0 / (obstacle - freespace));
		
		ssize_t const xbegin(m_proxy->GetXBegin());
		ssize_t const xend(m_proxy->GetXEnd());
		ssize_t const ybegin(m_proxy->GetYBegin());
		ssize_t const yend(m_proxy->GetYEnd());
		
    for (ssize_t ix(xbegin); ix < xend; ++ix)
      for (ssize_t iy(ybegin); iy < yend; ++iy) {
				int const value(m_proxy->GetValue(ix, iy));
				if (MINIMAL_DARK == color_code) {
					if (value > obstacle)
						glColor3d(1.0, 0.8, 0.8);
					else if (value == obstacle)
						glColor3d(0.6, 0.3, 0.3);
					else if (value > freespace)
						glColor3d(0.2, 0.2, 0.2);
					else
						glColor3d(0, 0, 0);
				}
				else {
					if (value > obstacle)
						glColor3d(0.5, 0, 1);
					else if (value == obstacle)
						glColor3d(0.5, 0, 0);
					else if (value < freespace)
						glColor3d(0, 0, 0.5);
					else if (value == freespace)
						glColor3d(0, 0.5, 0);
					else if (value < freespace)
						glColor3d(0, 0, 1);
					else {
						double const grey((value - freespace) * cscale);
						glColor3d(grey, grey, grey);
					}
				}
				glRectd(ix - 0.5, iy - 0.5, ix + 0.5, iy + 0.5);
      }
		
		//// grid frame origin and axes
		// 		glColor3d(1, 0, 0.5);
		// 		glBegin(GL_LINE_LOOP);
		// 		glVertex2d(0, 0);
		// 		glVertex2d(5 / m_proxy->GetDelta(), 0);
		// 		glVertex2d(0, 2 / m_proxy->GetDelta());
		// 		glEnd();
		
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
	}
	
	
	PtrTravProxy::
	PtrTravProxy(shared_ptr<TraversabilityMap const> travmap)
		: m_travmap(travmap)
	{
	}
	
	
	bool PtrTravProxy::
	Enabled() const
	{
		if( ! enable)
			return false;
    if( ! m_travmap)
      return false;
		return true;
	}
	
	
	double PtrTravProxy::
	GetX() const
	{
		return m_travmap->gframe.X();
	}
	
	
	double PtrTravProxy::
	GetY() const
	{
		return m_travmap->gframe.Y();
	}
	
	
	double PtrTravProxy::
	GetTheta() const
	{
		return m_travmap->gframe.Theta();
	}
	
	
	double PtrTravProxy::
	GetDelta() const
	{
		return m_travmap->gframe.Delta();
	}
	
	
	sfl::GridFrame const * PtrTravProxy::
	GetGridFrame()
	{
		return &(m_travmap->gframe);
	}
	
	
	int PtrTravProxy::
	GetObstacle() const
	{
		return m_travmap->obstacle;
	}
	
	
	int PtrTravProxy::
	GetFreespace() const
	{
		return m_travmap->freespace;
	}
	
	
	ssize_t PtrTravProxy::
	GetXBegin() const
	{
		return m_travmap->grid.xbegin();
	}
	
	
	ssize_t PtrTravProxy::
	GetXEnd() const
	{
		return m_travmap->grid.xend();
	}
	
	
	ssize_t PtrTravProxy::
	GetYBegin() const
	{
		return m_travmap->grid.ybegin();
	}
	
	
	ssize_t PtrTravProxy::
	GetYEnd() const
	{
		return m_travmap->grid.yend();
	}
	
	
	int PtrTravProxy::
	GetValue(ssize_t ix, ssize_t iy) const
	{
		try {
			return m_travmap->grid.at(ix, iy);
		}
		catch (out_of_range) {
		}
		return m_travmap->freespace;
	}
  
	
	RDTravProxy::
	RDTravProxy(boost::shared_ptr<sfl::RDTravmap> rdtravmap)
		: m_rdtravmap(rdtravmap)
	{
	}
	
	
	bool RDTravProxy::
	Enabled() const
	{
		if( ! enable)
			return false;
		return true;
	}
	
	
	double RDTravProxy::
	GetX() const
	{
		return m_rdtravmap->GetGridFrame().X();
	}
	
	
	double RDTravProxy::
	GetY() const
	{
		return m_rdtravmap->GetGridFrame().Y();
	}
	
	
	double RDTravProxy::
	GetTheta() const
	{
		return m_rdtravmap->GetGridFrame().Theta();
	}
	
	
	sfl::GridFrame const * RDTravProxy::
	GetGridFrame()
	{
		return &(m_rdtravmap->GetGridFrame());
	}
	
	
	double RDTravProxy::
	GetDelta() const
	{
		return m_rdtravmap->GetGridFrame().Delta();
	}
	
	
	int RDTravProxy::
	GetObstacle() const
	{
		return m_rdtravmap->GetObstacle();
	}
	
	
	int RDTravProxy::
	GetFreespace() const
	{
		return m_rdtravmap->GetFreespace();
	}
	
	
	ssize_t RDTravProxy::
	GetXBegin() const
	{
		return m_rdtravmap->GetXBegin();
	}
	
	
	ssize_t RDTravProxy::
	GetXEnd() const
	{
		return m_rdtravmap->GetXEnd();
	}
	
	
	ssize_t RDTravProxy::
	GetYBegin() const
	{
		return m_rdtravmap->GetYBegin();
	}
	
	
	ssize_t RDTravProxy::
	GetYEnd() const
	{
		return m_rdtravmap->GetYEnd();
	}
	
	
	int RDTravProxy::
	GetValue(ssize_t ix, ssize_t iy) const
	{
		int result(0);
		try {
			m_rdtravmap->GetValue(ix, iy, result);
		}
		catch (out_of_range) {
		}
		return result;
	}
	
	  
  TraversabilityCamera::
  TraversabilityCamera(const string & name,
												shared_ptr<TravProxyAPI> proxy)
    : Camera(name,
						 "bbox of traversability map"),
			m_proxy(proxy)
  {
  }
  
  
  TraversabilityCamera::
  TraversabilityCamera(const string & name,
											 TravProxyAPI * proxy)
    : Camera(name,
						 "bbox of traversability map"),
			m_proxy(proxy)
  {
  }
	
	
	void TraversabilityCamera::
	ConfigureView(View & view)
	{
		if ( ! m_proxy->Enabled()) {
			view.SetBounds(0, 0, 1, 1);
			return;
		}

		sfl::GridFrame const * gframe(m_proxy->GetGridFrame());
		if ( ! gframe) {
			view.SetBounds(0, 0, 1, 1);
			return;
		}
		
		sfl::GridFrame::position_t const
			p0(gframe->GlobalPoint(m_proxy->GetXBegin() - 1,
														 m_proxy->GetYBegin() - 1));		
		sfl::GridFrame::position_t const
			p1(gframe->GlobalPoint(m_proxy->GetXEnd(),
														 m_proxy->GetYEnd()));
		sfl::GridFrame::position_t const
			p2(gframe->GlobalPoint(m_proxy->GetXBegin() - 1,
														 m_proxy->GetYEnd()));
		sfl::GridFrame::position_t const
			p3(gframe->GlobalPoint(m_proxy->GetXEnd(),
														 m_proxy->GetYBegin() - 1));
		
    BBox bb(p0.v0, p0.v1, p1.v0, p1.v1);
		bb.Update(p2.v0, p2.v1);
		bb.Update(p3.v0, p3.v1);
		view.SetBounds(bb, 1);
	}
	
}
