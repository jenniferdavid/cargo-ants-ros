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


#ifndef PNF_HPP
#define PNF_HPP


#include <boost/shared_ptr.hpp>


namespace sfl {
  class GridFrame;
}


namespace pnf {
  class Flow;
}


namespace local {
  class poster;
}


namespace npm {

class PNF
{
public:
  typedef enum { NONE, ENVDIST, OBJDIST, ROBDIST, DONE } step_t;
  
  const double robot_x;
  const double robot_y;
  const double robot_r;
  const double robot_v;
  const double goal_x;
  const double goal_y;
  const double goal_r;
  const double grid_width;
  const size_t grid_wdim;
  const double resolution;

  PNF(double robot_x, double robot_y,
      double robot_r, double robot_v,
      double goal_x, double goal_y, double goal_r,
      double grid_width, size_t grid_wdim,
      bool enable_thread);

  bool AddStaticLine(double globx0, double globy0,
		     double globx1, double globy1);
  bool AddStaticObject(double globx, double globy);
  bool SetDynamicObject(size_t id, double globx, double globy,
			double r, double v);
  bool RemoveStaticObject(double globx, double globy);
  void RemoveDynamicObject(size_t id);
  
  void StartPlanning();
  step_t GetStep(bool fake_thread) const;
  
  boost::shared_ptr<const sfl::GridFrame> GetGridFrame() const
  { return m_frame; }
  
  boost::shared_ptr<pnf::Flow> GetFlow()
  { return m_flow; }

private:
  boost::shared_ptr<pnf::Flow> m_flow;
  boost::shared_ptr<sfl::GridFrame> m_frame;
  
  mutable boost::shared_ptr<local::poster> m_poster;
  
  void Wait();
};

}

#endif // PNF_HPP
