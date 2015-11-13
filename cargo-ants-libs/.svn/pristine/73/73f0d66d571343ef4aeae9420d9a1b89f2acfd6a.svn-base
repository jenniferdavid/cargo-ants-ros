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


#ifndef DODRAWING_HPP
#define DODRAWING_HPP


#include <npm/gfx/Drawing.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>


namespace sfl {
  class Frame;
  class HullIterator;
  class DistanceObjective;
  class HeadingObjective;
  class DynamicWindow;
  class RobotModel;
}


class DODrawing
  : public npm::Drawing
{
public:
  DODrawing(const std::string & name,
	    boost::shared_ptr<const sfl::DistanceObjective> distobj,
	    boost::shared_ptr<const sfl::HeadingObjective> headobj,
	    boost::shared_ptr<const sfl::DynamicWindow> dwa,
	    boost::shared_ptr<const sfl::RobotModel> rm);
  
  virtual void Draw();
  
private:
  boost::shared_ptr<const sfl::DistanceObjective> m_distobj;
  boost::shared_ptr<const sfl::HeadingObjective> m_headobj;
  boost::shared_ptr<const sfl::DynamicWindow> m_dwa;
  boost::shared_ptr<const sfl::RobotModel> m_rm;
  
  void DrawObstaclePaths(size_t iqdl, size_t iqdr);
  void DrawCollisionPrediction(size_t iqdl, size_t iqdr,
			       size_t igx, size_t igy);
  void DrawPose(const sfl::Frame & pose,
		double red, double green, double blue);
  void DrawPath();
};

#endif // DODRAWING_HPP
