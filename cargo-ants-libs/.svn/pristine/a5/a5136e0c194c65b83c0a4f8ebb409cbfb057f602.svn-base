/* -*- mode: C++; tab-width: 2 -*- */
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


#ifndef ESTAR_DRAWING_HPP
#define ESTAR_DRAWING_HPP


#include <npm/gfx/Drawing.hpp>
#include <npm/gfx/Camera.hpp>
#include <boost/shared_ptr.hpp>


namespace estar {
  class Facade;
}

namespace gfx {
  class ColorScheme;
}

namespace sfl {
  class GridFrame;
}

namespace npm {

class PlanProxy {
public:
	virtual ~PlanProxy() {}
  virtual const estar::Facade * GetFacade() = 0;
  virtual const sfl::GridFrame * GetFrame() = 0;
	virtual bool Enabled() const { return true; }
};


class EstarDrawing
  : public npm::Drawing
{
public:
  typedef enum { VALUE, RHS, META, QUEUE, UPWIND, OBST, STATUS } what_t;
  
  what_t what;
  
  EstarDrawing(const std::string & name,
							 boost::shared_ptr<PlanProxy> proxy,
							 what_t what);
  
  EstarDrawing(const std::string & name,
							 boost::shared_ptr<PlanProxy> proxy,
							 what_t what,
							 boost::shared_ptr<gfx::ColorScheme> custom_cs);
  
  virtual void Draw();
  
private:
  boost::shared_ptr<PlanProxy> m_proxy;
  boost::shared_ptr<gfx::ColorScheme> m_custom_cs;
};


class EstarCamera
	: public npm::Camera
{
public:
	EstarCamera(const std::string & name,
							boost::shared_ptr<PlanProxy> proxy);
	
	virtual void ConfigureView(npm::View & view);
	
protected:
	boost::shared_ptr<PlanProxy> m_proxy;
};

}

#endif // ESTAR_DRAWING_HPP
